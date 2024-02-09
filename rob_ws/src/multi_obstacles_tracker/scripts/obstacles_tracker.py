#!/usr/bin/env python
import rospy
import numpy as np
import datetime
from multi_obstacles_tracker_msgs.msg import ObstacleMeasureStampedArray 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32, Quaternion, Point
from costmap_converter.msg import ObstacleMsg, ObstacleArrayMsg
import tf
from stonesoup.predictor.kalman import KalmanPredictor, UnscentedKalmanPredictor, ExtendedKalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater, UnscentedKalmanUpdater, ExtendedKalmanUpdater
from stonesoup.models.transition.linear import CombinedLinearGaussianTransitionModel, ConstantVelocity
from stonesoup.models.measurement.linear import LinearGaussian
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.measures import Mahalanobis
from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.deleter.error import CovarianceBasedDeleter
from stonesoup.initiator.simple import MultiMeasurementInitiator
from stonesoup.types.state import GaussianState
from stonesoup.types.detection import Detection
from stonesoup.base import Property
from stonesoup.dataassociator import DataAssociator
from stonesoup.deleter import Deleter
from stonesoup.initiator import Initiator
from stonesoup.reader import DetectionReader
from stonesoup.tracker import Tracker
from stonesoup.updater import Updater


class RTMultiTargetTracker(Tracker):
    """A simple multi target tracker.

    Track multiple objects using Stone Soup components. The tracker works by
    first calling the :attr:`data_associator` with the active tracks, and then
    either updating the track state with the result of the :attr:`updater` if
    a detection is associated, or with the prediction if no detection is
    associated to the track. Tracks are then checked for deletion by the
    :attr:`deleter`, and remaining unassociated detections are passed to the
    :attr:`initiator` to generate new tracks.

    Parameters
    ----------
    """
    initiator: Initiator = Property(doc="Initiator used to initialise the track.")
    deleter: Deleter = Property(doc="Initiator used to initialise the track.")
    detector: DetectionReader = Property(doc="Detector used to generate detection objects.")
    data_associator: DataAssociator = Property(
        doc="Association algorithm to pair predictions to detections")
    updater: Updater = Property(doc="Updater used to update the track object to the new state.")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._tracks = set()
        self._last_association = {}

    @property
    def tracks(self):
        return self._tracks

    def __iter__(self):
        return self

    def __next__(self):
        """
        Returns
        -------
        : :class:`datetime.datetime`
            Datetime of current time step
        : set of :class:`~.Track`
            Tracks existing in the time step
        """
        raise NotImplementedError

    def update(self, time, detections):
        associations = self.data_associator.associate(self.tracks, detections, time)
        associated_detections = set()
        for track, hypothesis in associations.items():
            if hypothesis:
                state_post = self.updater.update(hypothesis)
                track.append(state_post)
                associated_detections.add(hypothesis.measurement)
            else:
                track.append(hypothesis.prediction)

        to_del_tracks = self.deleter.delete_tracks(self.tracks)
        for track in to_del_tracks:
            del self._last_association[track.id]
        self._tracks -= to_del_tracks

        self._tracks |= self.initiator.initiate(detections - associated_detections, time)

        for track in self._tracks:
            chosen_track = 0
            chosen_detection = None
            min_score = None
            for detection in detections:
                score = np.linalg.norm(np.array([[track.state_vector[0]], [track.state_vector[2]]]) - detection.state_vector)
                if min_score is None or score < min_score:
                    chosen_detection = detection
                    chosen_track = track.id
                    min_score = score
            self._last_association[chosen_track] = chosen_detection

        return time, self.tracks
    
class ObstaclesTracker:
    def __init__(self) -> None:
        # get constant velocity model as transition model of each kalman filter
        self.transition_model = CombinedLinearGaussianTransitionModel([ConstantVelocity(0.5), ConstantVelocity(0.5)])

        # get measurement model
        self.measurement_model = LinearGaussian(4, [0, 2], np.diag([0.25, 0.25]))

        # get kalman predictor model
        self.predictor = ExtendedKalmanPredictor(self.transition_model)

        # get kalman updater model
        self.updater = ExtendedKalmanUpdater(self.measurement_model)

        # get hypothesiser
        self.hypothesiser = DistanceHypothesiser(self.predictor, self.updater, measure=Mahalanobis(), missed_distance=3)

        # get data associator
        self.data_associator = GNNWith2DAssignment(self.hypothesiser)

        # get deleter
        self.deleter = CovarianceBasedDeleter(covar_trace_thresh=3)

        # get initiator
        s_prior_state = GaussianState([[0], [0], [0], [0]], np.diag([0.1, 1, 0.1, 1]))
        min_detections = 1

        self.initiator = MultiMeasurementInitiator(
            prior_state=s_prior_state,
            measurement_model=self.measurement_model,
            deleter=self.deleter,
            data_associator=self.data_associator,
            updater=self.updater,
            min_points=min_detections
        )

         # get tracker
        self.tracker = RTMultiTargetTracker(
            initiator=self.initiator,
            deleter=self.deleter,
            detector=None,
            data_associator=self.data_associator,
            updater=self.updater,
        )

        # subscribe to the topic /measures to get mean and cov of each detected obstacles
        rospy.Subscriber("/measures", ObstacleMeasureStampedArray, self.measuresCallback)

        # Publish to the topic /obstacles
        self.obstacles_array_msg_pub_ = rospy.Publisher("/obstacles", ObstacleArrayMsg, queue_size=1)

        # Publish to the topic /viz_obstacles
        self.viz_obstacles_array_msg_pub_ = rospy.Publisher("/viz_obstacles", Marker, queue_size=1)
        
        # spin it
        rospy.spin()

    def measuresCallback(self, measures):
        obstacles_array_msg = ObstacleArrayMsg()
        viz_obstacles_array_msg = Marker()

        time = datetime.datetime.fromtimestamp(measures.measures[0].header.stamp.secs + measures.measures[0].header.stamp.nsecs/10e6)
        detections = set()
        for measure in measures.measures:
            measurement_model = LinearGaussian(ndim_state=4, mapping=(0, 2), noise_covar=np.array([[measure.covariance[0], measure.covariance[1]], [measure.covariance[2], measure.covariance[3]]]))
            detection = Detection(state_vector=np.array([[measure.mean[0]], [measure.mean[1]]]), timestamp=time, measurement_model=measurement_model)
            detections.add(detection)
        self.tracker.update(time, detections)
        
        viz_obstacles_array_msg.header = measures.measures[0].header
        viz_obstacles_array_msg.ns = "multi_obstacles_tracker"
        viz_obstacles_array_msg.id = 0
        viz_obstacles_array_msg.type = Marker.LINE_LIST
        viz_obstacles_array_msg.action = Marker.ADD
        viz_obstacles_array_msg.pose.position.x = 0
        viz_obstacles_array_msg.pose.position.y = 0
        viz_obstacles_array_msg.pose.position.z = 0
        viz_obstacles_array_msg.color.a = 1
        viz_obstacles_array_msg.color.r = 1
        viz_obstacles_array_msg.color.g = 0.1
        viz_obstacles_array_msg.color.b = 0.1
        viz_obstacles_array_msg.scale.x = 0.05

        for track in self.tracker._tracks:
            listener = tf.TransformListener()

            try:
                (trans, rot) = listener.lookupTransform('base_link', 'map', rospy.Time.from_sec(measures.measures[0].header.stamp.to_sec()))
                print(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e.msg)
                continue
            obstacle_msg = ObstacleMsg()

            obstacle_msg.header = measures.measures[0].header

            a = self.tracker._last_association[track.id].measurement_model.noise_covar[0, 0]
            b = self.tracker._last_association[track.id].measurement_model.noise_covar[0, 1]
            c = self.tracker._last_association[track.id].measurement_model.noise_covar[1, 0]
            
            lambda_1 = np.abs((a+c)/2 + np.sqrt(((a-c)**2)/4 + b**2))
            lambda_2 = np.abs((a+c)/2 - np.sqrt(((a-c)**2)/4 + b**2))
            theta = 0
            if b == 0 and a >= c:
                theta = 0
            elif b == 0 and a < c:
                theta = np.pi/2
            else:
                theta = np.arctan2(lambda_1-a, b)
            
            OA = 1.96 * np.sqrt(lambda_1) * np.array([np.cos(theta), np.sin(theta)])
            OB = 1.96 * np.sqrt(lambda_2) * np.array([np.sin(theta), -np.cos(theta)])

            A1 = OA + OB
            A2 = OA - OB
            A3 = -OA - OB
            A4 = -OA + OB

            A_1_pos_point = Point32()
            A_1_pos_point.x = track.state_vector[0] + A1[0] 
            A_1_pos_point.y = track.state_vector[2] + A1[1]
            A_1_pos_point.z = 0

            A_2_pos_point = Point32()
            A_2_pos_point.x = track.state_vector[0] + A2[0]
            A_2_pos_point.y = track.state_vector[2] + A2[1]
            A_2_pos_point.z = 0

            A_3_pos_point = Point32()
            A_3_pos_point.x = track.state_vector[0] + A3[0]
            A_3_pos_point.y = track.state_vector[2] + A3[1]
            A_3_pos_point.z = 0

            A_4_pos_point = Point32()
            A_4_pos_point.x = track.state_vector[0] + A4[0]
            A_4_pos_point.y = track.state_vector[2] + A4[1]
            A_4_pos_point.z = 0

            obstacle_msg.polygon.points.append(A_1_pos_point)
            obstacle_msg.polygon.points.append(A_2_pos_point)
            obstacle_msg.polygon.points.append(A_3_pos_point)
            obstacle_msg.polygon.points.append(A_4_pos_point)

            quat = Quaternion()
            quat.x = 0
            quat.y = 0
            quat.z = 0
            quat.w = 1
            obstacle_msg.orientation = quat

            obstacle_msg.velocities.twist.linear.x = track.state_vector[1]
            obstacle_msg.velocities.twist.linear.y = track.state_vector[3]
            obstacle_msg.velocities.twist.linear.z = 0
            obstacle_msg.velocities.twist.angular.x = 0
            obstacle_msg.velocities.twist.angular.y = 0
            obstacle_msg.velocities.twist.angular.z = 0

            obstacle_msg.radius = 2

            obstacles_array_msg.obstacles.append(obstacle_msg)

            viz_obstacles_array_msg.points.append(A_1_pos_point)
            viz_obstacles_array_msg.points.append(A_2_pos_point)
            viz_obstacles_array_msg.points.append(A_2_pos_point)
            viz_obstacles_array_msg.points.append(A_3_pos_point)
            viz_obstacles_array_msg.points.append(A_3_pos_point)
            viz_obstacles_array_msg.points.append(A_4_pos_point)
            viz_obstacles_array_msg.points.append(A_4_pos_point)
            viz_obstacles_array_msg.points.append(A_1_pos_point)

        if len(obstacles_array_msg.obstacles) > 0:
            self.obstacles_array_msg_pub_.publish(obstacles_array_msg)
            self.viz_obstacles_array_msg_pub_.publish(viz_obstacles_array_msg)

if __name__ == "__main__":
    rospy.init_node('obstacles_tracker')

    try:
        ne = ObstaclesTracker()
    except rospy.ROSInterruptException:
        pass
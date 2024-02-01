#!/usr/bin/env python
import rospy
import numpy as np
import datetime
from multi_obstacles_tracker_msgs.msg import ObstacleMeasureStampedArray, ObstacleMeasureStamped 
from stonesoup.predictor.kalman import KalmanPredictor
from stonesoup.updater.kalman import KalmanUpdater
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
        associations = self.data_associator.associate(
            self.tracks, detections, time)
        associated_detections = set()
        for track, hypothesis in associations.items():
            if hypothesis:
                state_post = self.updater.update(hypothesis)
                track.append(state_post)
                associated_detections.add(hypothesis.measurement)
            else:
                track.append(hypothesis.prediction)

        self._tracks -= self.deleter.delete_tracks(self.tracks)
        self._tracks |= self.initiator.initiate(
            detections - associated_detections, time)

        return time, self.tracks
    
class ObstaclesTracker:
    def __init__(self) -> None:
        # get constant velocity model as transition model of each kalman filter
        self.transition_model = CombinedLinearGaussianTransitionModel([ConstantVelocity(0.05), ConstantVelocity(0.05)])

        # get measurement model
        self.measurement_model = LinearGaussian(4, [0, 2], np.diag([0.25, 0.25]))

        # get kalman predictor model
        self.predictor = KalmanPredictor(self.transition_model)

        # get kalman updater model
        self.updater = KalmanUpdater(self.measurement_model)

        # get hypothesiser
        self.hypothesiser = DistanceHypothesiser(self.predictor, self.updater, measure=Mahalanobis(), missed_distance=3)

        # get data associator
        self.data_associator = GNNWith2DAssignment(self.hypothesiser)

        # get deleter
        self.deleter = CovarianceBasedDeleter(covar_trace_thresh=2)

        # get initiator
        s_prior_state = GaussianState([[0], [0], [0], [0]], np.diag([0, 0.5, 0, 0.5]))
        min_detections = 3

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

        # subscribe to the topic /obstacles to get mean and cov of each detected obstacles
        rospy.Subscriber("/measures", ObstacleMeasureStampedArray, self.measuresCallback)
        
        # spin it
        rospy.spin()

    def measuresCallback(self, measures):
        time = datetime.datetime.utcfromtimestamp(measures.measures[0].header.stamp)
        detections = set()

        for measure in measures.measures:
            measurement_model = LinearGaussian(ndim_state=4, mapping=(0, 2), noise_covar=np.array([[measure.covariance[0], measure.covariance[1]], [measure.covariance[2], measure.covariance[3]]]))
            detection = Detection(state_vector=np.array([[measure.mean[0]], [measure.mean[1]]]), timestamp=time, measurement_model=measurement_model)
            detections.add(detection)
            self.tracker.update(time, detections)
        
        print(self.tracker.tracks)

if __name__ == "__main__":
    rospy.init_node('obstacles_tracker')

    try:
        ne = ObstaclesTracker()
    except rospy.ROSInterruptException:
        pass
#include "ros/ros.h"

#include "multi_obstacles_tracker_msgs/ClusterStamped.h"
#include "multi_obstacles_tracker_msgs/ClusterStampedArray.h"

#include "kalman_ros/kalman.hpp"

#include <sstream>
#include <list>


class ObstaclesTracker
{
public:
    // ROS variables
    ros::NodeHandle n_;

    // publisher variables
    // ros::Publisher clusters_pub_;

    // subscriber variables
    ros::Subscriber clusters_sub_;

    // kalman filter vector
    std::vector<KalmanFilter> filters_;
    std::vector<double> filters_lifes_;

    // thresholds
    double distance_threshold = 0.1;
    double time_threshold = 15;
    

    ObstaclesTracker(ros::NodeHandle n) : n_(n)
    {
        // publish to the /cluster topic
        // clusters_pub_ = n_.advertise<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10);

        // subscribe to the /scan_filtered topic
        clusters_sub_ = n_.subscribe<multi_obstacles_tracker_msgs::ClusterStampedArray>("/cluster", 10, &ObstaclesTracker::clusterCallback, this);
    }

    void clusterCallback(const multi_obstacles_tracker_msgs::ClusterStampedArray::ConstPtr& clusters_in)
    {
        // if there is not KF active
        if(filters_.empty())
        {
            for(unsigned i = 0; i < clusters_in->clusters.size(); i++)
            {
                // add new KF per cluster
                ObstaclesTracker::addKalmanFilter(clusters_in->clusters[i]);
            }
        }
        else
        {
            // get the cost matrix which computes the distance between the barycenter of the cloud and the predicted pos from the KF
            Eigen::MatrixXd cost_matrix = ObstaclesTracker::computeCostMatrix(clusters_in);

            // get the threshold matrix
            Eigen::MatrixXd threshold_matrix = ObstaclesTracker::computeThresholdMatrix(cost_matrix, distance_threshold); 

            // process the threshold matrix
            // first row by row
            std::vector<KalmanFilter> tmp_filters_;
            std::vector<double> tmp_filters_lifes_;
            Eigen::ArrayXi del_mask = Eigen::ArrayXi::Zero(threshold_matrix.rows());
            for(unsigned i = 0; i < threshold_matrix.rows(); i++)
            {
                bool match_found = false;
                for(unsigned j = 0; j < threshold_matrix.cols(); j++)
                {
                    if(threshold_matrix(i, j) != -1)
                    {
                        // update the i-th filter. Here, we take the timestamp of the first cluster because they all have the same header.
                        double dt = ((double) clusters_in->clusters[0].header.stamp.sec + (double) clusters_in->clusters[0].header.stamp.nsec/1e-9) - filters_[i].time();

                        Eigen::Matrix4d A;
                        A << 1, dt, 0, 0, 
                            0, 1, 0, 0, 
                            0, 0, 1, dt, 
                            0, 0, 0, 1;

                        Eigen::Vector2d y;
                        y << clusters_in->clusters[j].barycenter.x, clusters_in->clusters[j].barycenter.y;

                        filters_[i].predict_and_update(y, dt, A);
                        filters_lifes_[i] = 0;
                        match_found = true;
                        break;
                    }
                }

                if(!match_found)
                {
                    // update life of the fitler. Here, we take the timestamp of the first cluster because they all have the same header.
                    double dt = ((double) clusters_in->clusters[0].header.stamp.sec + (double) clusters_in->clusters[0].header.stamp.nsec/1e-9) - filters_[i].time();
                    filters_lifes_[i] += dt;

                    if(filters_lifes_[i] < time_threshold)
                    {
                        tmp_filters_.push_back(filters_[i]);
                        tmp_filters_lifes_.push_back(filters_lifes_[i]);
                    }
                }
            }

            filters_ = tmp_filters_;
            filters_lifes_ = tmp_filters_lifes_;

            // then col by col
            for(unsigned i = 0; i < threshold_matrix.cols(); i++)
            {
                bool match_found = false;
                for(unsigned j = 0; j < threshold_matrix.rows(); j++)
                {
                    if(threshold_matrix(j, i) != -1)
                    {
                        match_found = true;
                        break;
                    }
                }
                if(!match_found)
                {
                    // create new KF for the unmatched cloud
                    ObstaclesTracker::addKalmanFilter(clusters_in->clusters[i]);
                }
            }
        }

        for(unsigned i = 0; i < filters_.size(); i++)
        {
            auto x_hat = filters_[i].state();
            ROS_INFO("object %d with life %f s: x_x %f; v_x %f; x_y %f; v_y %f; t %f", i, filters_lifes_[i], x_hat(0), x_hat(1), x_hat(2), x_hat(3), filters_[i].time());
        }
        ROS_INFO("\n\n");
    }

    void addKalmanFilter(const multi_obstacles_tracker_msgs::ClusterStamped cluster_in)
    {
        // create the KF   
        Eigen::Matrix4d A = Eigen::Matrix4d::Identity();

        Eigen::MatrixXd C(2, 4);
        C << 1, 0, 0, 0,
            0, 0, 1, 0;

        // q represents the deviation of the velocity (in (m/s)^2)
        double q = 1.0;
        Eigen::Matrix4d Q;
        Q << 0, 0, 0, 0,
             0, q, 0, 0,
             0, 0, 0, 0, 
             0, 0, 0, q; 

        // r represents the deviation of the position measure (in m^2)
        double r = 0.25;
        Eigen::Matrix2d R;
        R << r, 0,
             0, r;

        Eigen::Matrix4d P;
        P << r, 0, 0, 0,
             0, 10*q, 0, 0,
             0, 0, r, 0, 
             0, 0, 0, 10*q; 

        KalmanFilter kf = KalmanFilter(0, A, C, Q, R, P);
        
        // init the KF at the current barycenter of the cluster and with a null velocity
        kf.init(((double) cluster_in.header.stamp.sec + (double) cluster_in.header.stamp.nsec/1e9), Eigen::Vector4d(cluster_in.barycenter.x, 0, cluster_in.barycenter.y, 0));

        // add the KF to the vector
        filters_.push_back(kf);

        // add life KF to the vector
        filters_lifes_.push_back(0.0f);
    }

    Eigen::MatrixXd computeCostMatrix(const multi_obstacles_tracker_msgs::ClusterStampedArray::ConstPtr& clusters_in)
    {   
        // create the cost matrix of size N, M with N the number of filters and M the number of clusters
        Eigen::MatrixXd cost_matrix(filters_.size(), clusters_in->clusters.size());

        // fill the cost matrix
        for(unsigned i = 0; i < filters_.size(); i++)
        {
            // predict the state of the i-th filter at k+1 instant
            double dt = ((double) clusters_in->clusters[0].header.stamp.sec + (double) clusters_in->clusters[0].header.stamp.nsec/1e9) - filters_[i].time();
            Eigen::Matrix4d A;
            A << 1, dt, 0, 0, 
                 0, 1, 0, 0, 
                 0, 0, 1, dt, 
                 0, 0, 0, 1;
            Eigen::VectorXd pred_state = A * filters_[i].state();

            for(unsigned j = 0; j < clusters_in->clusters.size(); j++)
            {
                // fill each element of the matix by the L1 distance between the barycenter of the point cloud and the predicted pos by the KF
                cost_matrix(i, j) = ObstaclesTracker::computeL1Distance(clusters_in->clusters[j].barycenter.x, clusters_in->clusters[j].barycenter.y, 
                                                                        pred_state(0), pred_state(2));
            }
        }
        return cost_matrix;
    }

    Eigen::MatrixXd computeThresholdMatrix(const Eigen::MatrixXd cost_matrix, double threshold)
    {   
        Eigen::MatrixXd threshold_matrix = Eigen::MatrixXd::Constant(cost_matrix.rows(), cost_matrix.cols(), -1);
        for(unsigned i = 0; i < cost_matrix.rows(); i++)
        {
            Eigen::Index min_col;
            double min = cost_matrix.row(i).array().minCoeff(&min_col);
            if(min <= threshold)
            {
                threshold_matrix(i, min_col) = min;
            }
        }
        return threshold_matrix;
    }

    double computeL1Distance(const double x1, const double y1, const double x2, const double y2)
    {
        return std::abs(x2 - x1) + std::abs(y2 - y1); 
    }

};


int main(int argc, char** argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "obstacles_tracking");

    // create a ROS node handle
    ros::NodeHandle n;

    // create scan filter
    ObstaclesTracker ot(n);

    // loop ros
    ros::spin();

    return 0;
}

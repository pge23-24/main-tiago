#include <cmath>

#include <ros/ros.h>

#include "std_msgs/String.h"
#include <sstream>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

int main(int argc, char* argv[]){
    // try{ // add the try catch statement
        // This must be called before anything else ROS-related
        ros::init(argc, argv, "dynamic_obstacle_node");

        // Create a ROS node handle
        ros::NodeHandle nh;

        // TO MODIFY
        ros::Publisher dyn_obstacles_pub = 
        nh.advertise<costmap_converter::ObstacleArrayMsg>("/test_optim_node/obstacles", 1);

        // dynamic obstacle core
        float y_0 = -3.0;
        float vel_x = 0.0;
        float vel_y = 0.8;
        float range_y = 6.0;

        costmap_converter::ObstacleArrayMsg obstacle_msg = costmap_converter::ObstacleArrayMsg();
        obstacle_msg.header.stamp = ros::Time::now();
        obstacle_msg.header.frame_id = "odom"; // CHANGE HERE: odom/map

        // add point obstacle
        obstacle_msg.obstacles.push_back(costmap_converter::ObstacleMsg());
        geometry_msgs::Point32 P;
        P.x = -1.5;
        P.y = 0;
        P.z = 0;
        obstacle_msg.obstacles[0].polygon.points.push_back(P);

        auto yaw = atan2(vel_y, vel_x);
        tf::Quaternion q_tf;
        q_tf.setRPY(0, 0, yaw);
        q_tf.normalize();

        geometry_msgs::Quaternion q_msg; 
        quaternionTFToMsg(q_tf , q_msg);

        obstacle_msg.obstacles[0].orientation = q_msg;

        obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_x;
        obstacle_msg.obstacles[0].velocities.twist.linear.y = vel_y;
        obstacle_msg.obstacles[0].velocities.twist.linear.z = 0;
        obstacle_msg.obstacles[0].velocities.twist.angular.x = 0;
        obstacle_msg.obstacles[0].velocities.twist.angular.y = 0;
        obstacle_msg.obstacles[0].velocities.twist.angular.z = 0;

        ros::Rate ros_rate(10);
        double t = 0.0;
        while(ros::ok()){
            // vary y component of the point obstacle
            if(vel_y >= 0){
                obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + fmod((vel_y*t), range_y);
            }else{
                obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + fmod((vel_y*t), range_y) - range_y;
            }

            t += 0.1;

            dyn_obstacles_pub.publish(obstacle_msg);

            ros_rate.sleep();
        }

    // }catch(ros::ROSInterruptException){
    //     // do nothing
    // }
}
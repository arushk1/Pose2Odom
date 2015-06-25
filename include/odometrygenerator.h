#ifndef ODOMETRYGENERATOR_H
#define ODOMETRYGENERATOR_H

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>

class OdometryGenerator
{
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    geometry_msgs::Pose init_pose;
    tf::TransformBroadcaster my_tf_broadcaster;
    tf::TransformListener listener;

    int once;

public:
    OdometryGenerator(ros::NodeHandle &n);
    ~OdometryGenerator();

    void MocapDataCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
};

#endif // ODOMETRYGENERATOR_H

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Eigen>
#include "./../include/odometrygenerator.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose2odom");
  ros::NodeHandle n;
  OdometryGenerator odom_generator(n);
  ros::spin();
  return 0;
}

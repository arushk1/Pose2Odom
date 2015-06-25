#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;

void callback(geometry_msgs::PoseStamped msg){

  tf::TransformBroadcaster cam_obj_br;
  tf::TransformListener obj_listener;
  tf::TransformListener cam_listener;

  tf::StampedTransform world_obj;
  tf::StampedTransform world_cam;
  tf::StampedTransform cam_obj;


  try {
    obj_listener.lookupTransform("/world", "/cal_obj/base_link", ros::Time(0), world_obj);
    cam_listener.lookupTransform("/world", "/rgbd_sensor/base_link", ros::Time(0), world_cam);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }

  cam_obj.getOrigin() = world_obj.getOrigin() - world_cam.getOrigin();
  cam_obj.getRotation() = world_obj.getRotation() - world_cam.getRotation();

  cam_obj_br.tf::StampedTransform(cam_obj, ros::Time::now(), "world", "cam2obj"));

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "calibration");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/optitrack/cal_obj/pose", 1000, callback);
  ros::spin();
  return 0;
}

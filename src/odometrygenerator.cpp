#include "./../include/odometrygenerator.h"

using namespace std;

OdometryGenerator::OdometryGenerator(ros::NodeHandle &n)
{
    once = 0;
    pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
    sub = n.subscribe("/optitrack/pixhawk/pose", 1, &OdometryGenerator::MocapDataCallback, this);
}

OdometryGenerator::~OdometryGenerator() {

}

void OdometryGenerator::MocapDataCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {

    if (once <1 ){
      init_pose = msg->pose;
      once++;
    }

    ros::Time msg_time = msg->header.stamp;
    geometry_msgs::Pose current_odom_pose;
    {
      // current_odom_pose.position.y = init_pose.position.x - msg->pose.position.x;
      // current_odom_pose.position.x = msg->pose.position.y - init_pose.position.y;
      // current_odom_pose.position.z = msg->pose.position.z - init_pose.position.z;
      geometry_msgs::Point current_position_w;
      current_position_w.x = msg->pose.position.x - init_pose.position.x;
      current_position_w.y = msg->pose.position.y - init_pose.position.y;
      current_position_w.z = msg->pose.position.z - init_pose.position.z;
      current_odom_pose.position.x = (+1.0) * current_position_w.x;
      current_odom_pose.position.y = (+1.0) * current_position_w.y;
      current_odom_pose.position.z = (+1.0) * current_position_w.z;
    }

    Eigen::Quaternion<double> init_quat(init_pose.orientation.w,
                                        init_pose.orientation.x,
                                        init_pose.orientation.y,
                                        init_pose.orientation.z);
    Eigen::Quaternion<double> msg_quat( msg->pose.orientation.w,
                                        msg->pose.orientation.x,
                                        msg->pose.orientation.y,
                                        msg->pose.orientation.z);
    {
      Eigen::Quaternion<double> current_quat_w = init_quat.inverse() * msg_quat; // TODO: not sure about this operation
      current_odom_pose.orientation.x = (+1.0) * current_quat_w.x();
      current_odom_pose.orientation.y = (+1.0) * current_quat_w.y();
      current_odom_pose.orientation.z = (+1.0) * current_quat_w.z();
      current_odom_pose.orientation.w = current_quat_w.w();
    }

    // publish current_odom_pose as geometry_msgs::TransformStamped
    geometry_msgs::TransformStamped drone2map_tf_msg;
    drone2map_tf_msg.header.stamp = msg_time;
    drone2map_tf_msg.header.frame_id = "odom";
    drone2map_tf_msg.child_frame_id  = "pixhawk/odom/base_link";

    drone2map_tf_msg.transform.translation.x = current_odom_pose.position.x;
    drone2map_tf_msg.transform.translation.y = current_odom_pose.position.y;
    drone2map_tf_msg.transform.translation.z = current_odom_pose.position.z;
    drone2map_tf_msg.transform.rotation.w = current_odom_pose.orientation.w;
    drone2map_tf_msg.transform.rotation.x = current_odom_pose.orientation.x;
    drone2map_tf_msg.transform.rotation.y = current_odom_pose.orientation.y;
    drone2map_tf_msg.transform.rotation.z = current_odom_pose.orientation.z;

    my_tf_broadcaster.sendTransform(drone2map_tf_msg);

    // publish current_odom_pose as geometry_msgs::PoseWithCovariance
    nav_msgs::Odometry odom;
    odom.header.stamp = msg_time;
    odom.header.frame_id = "/odom";
    odom.child_frame_id  = "pixhawk/odom/base_link";
    odom.pose.pose = current_odom_pose;
    pub.publish(odom);

    // publish world>map as geometry_msgs::TransformStamped
    tf::Transform world2map_tf;
    tf::StampedTransform world2drone_tf;
    tf::StampedTransform drone2map_tf;
    try{
        listener.lookupTransform("pixhawk/odom/base_link", "map",
                                 ros::Time(0), drone2map_tf);
        listener.lookupTransform("world", "pixhawk/base_link",
                                 ros::Time(0), world2drone_tf);
        world2map_tf = world2drone_tf * drone2map_tf;

        geometry_msgs::TransformStamped world2map_tf_msg;
        world2map_tf_msg.header.stamp = msg_time;
        world2map_tf_msg.header.frame_id = "world";
        world2map_tf_msg.child_frame_id  = "map";
        world2map_tf_msg.transform.translation.x = world2map_tf.getOrigin().x();
        world2map_tf_msg.transform.translation.y = world2map_tf.getOrigin().y();
        world2map_tf_msg.transform.translation.z = world2map_tf.getOrigin().z();
        world2map_tf_msg.transform.rotation.w = world2map_tf.getRotation().w();
        world2map_tf_msg.transform.rotation.x = world2map_tf.getRotation().x();
        world2map_tf_msg.transform.rotation.y = world2map_tf.getRotation().y();
        world2map_tf_msg.transform.rotation.z = world2map_tf.getRotation().z();

        my_tf_broadcaster.sendTransform(world2map_tf_msg);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
}

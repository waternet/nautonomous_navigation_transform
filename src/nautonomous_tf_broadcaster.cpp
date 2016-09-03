#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

tf::Quaternion quaternion;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    quaternion = tf::Quaternion(0,0,msg->orientation.z+0.327,msg->orientation.w-0.268);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    static tf::TransformBroadcaster br;
    ROS_INFO("received pose");
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
    transform.setRotation(quaternion);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "nautonomous_tf_publisher");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/gps_odom", 100, &poseCallback);
  ros::Subscriber sub2 = n.subscribe("/mavros/imu/data", 100, &imuCallback);
  tf::TransformBroadcaster broadcaster;

  ros::Rate rate(100);

  while(ros::ok()){
     broadcaster.sendTransform(
          tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
          tf::Vector3(1.0, 0.0, 0.5)), ros::Time::now(),
           "base_link", "camera_link"));

broadcaster.sendTransform(
          tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
          tf::Vector3(0.2, -0.2, 1.0)), ros::Time::now(),
           "base_link", "gps_link"));

broadcaster.sendTransform(
          tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
          tf::Vector3(-0.2, 0.5, 0.5)), ros::Time::now(),
           "base_link", "imu_link"));
    broadcaster.sendTransform(
          tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
          tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),
           "base_link", "base_footprint"));
    broadcaster.sendTransform(
          tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
          tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),
           "map", "odom_combined"));
    /*broadcaster.sendTransform(
		tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
        tf::Vector3(0.0, -0.5, 0.2)), ros::Time::now(),
         "base_footprint", "pixhawk_frame"));
    broadcaster.sendTransform(
        tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
        tf::Vector3(-0.2, 0.2, 0.4)), ros::Time::now(),
         "base_footprint", "gps_frame"));
    broadcaster.sendTransform(
        tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
        tf::Vector3(0.0, -0.5, 0.3)), ros::Time::now(),
         "base_footprint", "gps2_frame"));
    broadcaster.sendTransform(
        tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
        tf::Vector3(0.0, 0.5, 0.3)), ros::Time::now(),
         "base_footprint", "camera_frame"));*/
	rate.sleep();
    ros::spinOnce();
  }

  ros::spin();
  return 0;
}

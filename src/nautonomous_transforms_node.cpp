#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

tf::Quaternion quaternion;

//Subscribe to the IMU topic and extract the orientation
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    quaternion = tf::Quaternion(0,0,msg->orientation.z+0.327,msg->orientation.w-0.268); //TODO remove MAGIC numbers
}

//Subscribe to the pose topic and extract the rotation and position
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
    transform.setRotation(quaternion);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "map"));
}

// Is now done by the tf nodes from the launch file.
// void publishTransforms(tf::TransformBroadcaster broadcaster){
//     //Publish every 0.01 sec.
//   ros::Rate rate(100);

//   while(ros::ok()){
//      //Set manual transforms for the robot frame

//      //TODO have a configuration file determine the links and vector connections between transformations.
//      broadcaster.sendTransform(
//           tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
//           tf::Vector3(1.0, 0.0, 0.5)), ros::Time::now(),
//            "base_link", "camera_link"));

//     broadcaster.sendTransform(
//           tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
//           tf::Vector3(0.2, -0.2, 1.0)), ros::Time::now(),
//            "base_link", "gps_link"));

//     broadcaster.sendTransform(
//           tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
//           tf::Vector3(-0.2, 0.5, 0.5)), ros::Time::now(),
//            "base_link", "imu_link"));
//     broadcaster.sendTransform(
//           tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
//           tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),
//            "base_link", "base_footprint"));
//     broadcaster.sendTransform(
//           tf::StampedTransform( tf::Transform(tf::Quaternion(0, 0, 0 , 1),
//           tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(),
//            "map", "odom_combined"));

// 	rate.sleep();
//     ros::spinOnce();
//   }
//

int main(int argc, char** argv){
  ros::init(argc, argv, "nautonomous_tf_publisher");

  ros::NodeHandle n;
  
  //Subscribe to GPS and IMU topic
  ros::Subscriber odometrySubscriber = n.subscribe("odom", 100, &poseCallback);
  ros::Subscriber imuSubscriber = n.subscribe("imu", 100, &imuCallback);

  tf::TransformBroadcaster broadcaster;

  // this is now done by the tf broadcaster in the launchfile.
  //publishTransforms(broadcaster);
  ros::spin();

  return 0;
}

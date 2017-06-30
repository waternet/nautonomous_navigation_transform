// ROS
#include <ros/ros.h>
// 	Messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// 	Transforms
#include <tf/transform_broadcaster.h>

tf::Quaternion quaternion;

/**
 *\brief Subscribe to the IMU topic and extract the orientation
 *\param sensor_msgs::Imu msg
 *\return
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

/**
 *\brief Subscribe to the pose topic and extract the rotation and position. Send transform(gps+imu) to TransformBroadcaster 
 *\param nav_msgs::Odometry msg
 *\return
 */
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);

/*
 *\brief Main for nautonomous navigation transform. Subscribes to GPS and IMU topic.
 *\param
 *\return
 */
int main(int argc, char** argv);

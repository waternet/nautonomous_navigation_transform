#include <nautonomous_tf/nautonomous_tf_node.h>

/**
 *\brief Subscribe to the IMU topic and extract the orientation
 *\param sensor_msgs::Imu msg
 *\return
 */
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	quaternion = tf::Quaternion(0.0, 0.0, msg->orientation.z, msg->orientation.w);
}

/**
 *\brief Subscribe to the pose topic and extract the rotation and position. Send transform(gps+imu) to TransformBroadcaster 
 *\param nav_msgs::Odometry msg
 *\return
 */
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//ROS_INFO("Quaternion: %.2f", );

	if(quaternion.getW() != 0.000 || quaternion.getZ() != 0.000)
	{
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );
		transform.setRotation(quaternion);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom_combined", "base_link"));
	}

}

/*
 *\brief Main for nautonomous navigation transform. Subscribes to GPS and IMU topic.
 *\param
 *\return
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "nautonomous_tf_node");

	ros::NodeHandle n;

	//Subscribe to GPS and IMU topic
	ros::Subscriber odometrySubscriber = n.subscribe("odom", 100, &poseCallback);
	ros::Subscriber imuSubscriber = n.subscribe("imu", 100, &imuCallback);

	tf::TransformBroadcaster broadcaster;

	ros::spin();

	return 0;
}

# nautonomous tf {#nautonomous_tf}
Combines gps_odom+imu to create a transform for /tf.


nautonomous_tf_node.cpp

##Nodes
nautonomous_tf_node

/camera_link_broadcaster <br />
/footprint_link_broadcaster <br />
/gps_link_broadcaster <br />
/imu_link_broadcaster <br />
/odom_combined_broadcaster

##Topics
###Subscribe
/gps/odom
<br />
/imu/razor

###Publish
/tf

## Files
[Src](dir_5f483fb73a60372554ae375b13d59e7b.html)

##Overview
![launch_nautonomous_transforms.launch](../images/launch_nautonomous_transforms.png)
<br />
![legenda](../images/legenda.png)

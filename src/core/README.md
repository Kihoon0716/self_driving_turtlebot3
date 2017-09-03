# core 

The purpose of node is controller. This node send information from the others. And this node give driving information to lane follower node.

This node get 2 function

 1. monitoring
	* It is doing monitoring about the others node
 2. command
	* It is command to lane follower node in the case (ex).


#### subscribed topic
 * /image_calibrated_compressed(sensor_msgs/CompressedImage)
 * /image_calibrated(sensor_msgs/Image)
 * /stop_bar(self_driving_turtlebot3/Stop_bar)
 * /traffic_light(self_driving_turtlebot3/traffic_ligt)
 * /parking(std_msgs/String)
 * /scan(sensor_msgs/LaserScan)
 * /maze(std_msgs/String)
 * /signal_sign(std_msgs/string)
 * /objects(std_msgs/Float32MultiArray)

#### published topic
 * /command_lane_follower(std_msgs/String)


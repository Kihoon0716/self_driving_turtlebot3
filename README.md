## Requirements

* Python3.5
* Ubuntu 16.04
* [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* [Tensorflow](https://www.tensorflow.org/install/)
* [find_object_2d (ROS package)](http://wiki.ros.org/find_object_2d)
* [usb_cam (ROS package)](http://wiki.ros.org/usb_cam)


## Hardware
 * [Fish eye camera](https://ko.aliexpress.com/item/2mp-hd-1-3-CMOS-AR0330-H-264-mini-cmos-fpv-180-degree-wide-angle-fisheye/32793788459.html?trace=msiteDetail2pcDetail)
 * [Intel Joule](https://software.intel.com/en-us/iot/hardware/joule/dev-kit)
 * [Turtlebot3 burger](http://en.robotis.com/index/product.php?cate_code=111510)
## Usage

1. Permission
You should do
```
chmod +x "file name"
```
for all the files inside of src forder.

2. Path of machine learning data 
In the "signal_sign_detection.py"
```
self.saver.restore(self.sess, "/home/kihoon/catkin_ws/src/self_driving_turtlebot3/src/signal_sign_detection/model/model.ckpt")
```
Change this line for your workspace environment

3. catkin make
```
cd ~/catkin_ws && catkin_make
```
4. roscore and roslaunch
```
roscore
roslaunch self_driving_turtlebot3 self_driving.launch
```


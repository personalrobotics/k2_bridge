k2_client
=========
A client application for Kinect for Windows v2.

This software is a part of software package for integrating Kinect for Windows v2 with ROS. The software package is divided into two parts. One part runs on a Windows machine and streams data over the network, while the other part runs on a linux machine which reads the stream and publishes appropriate ROS topics.

This software is meant to be run on the linux side of the system. The corresponding package to run on the windows side is called `k2_server` and can be found at: https://github.com/personalrobotics/k2_server/releases

Setting up the software
=======================

1. Add the the k2_client package from following link to your ROS catkin workspace.

   https://github.com/personalrobotics/k2_client/releases

2. Edit the value of the parameter "serverHostname" with the IP address or hostname of your windows machine which is running `k2_server`. Once done, use `catkin_make` or `catkin_build` (preferred) to build the package.

3. Start all the ROS nodes by running the following command. Make sure that the roscore is running and the environment variable $ROS_MASTER_URI points to it.

   `roslaunch k2_client k2_client.launch`

# k2_bridge

This software is a part of software package for integrating kinect for windows v2 with ROS. The software package is divided into two parts. One part runs on windows machine and dumps the data over the network, while, the other part runs on linux machine which reads the stream and publishes appropriate ROS topics.

* [k2_server](k2_server) - Windows service to publish Kinect 2 data.
* [k2_client](k2_client) - Linux ROS package to subscribe to Kinect 2 data.

----

## License

`k2_bridge` is licensed under a BSD license. See the [LICENSE](k2_client/LICENSE.md) for more information.

## Contributors

`k2_bridge` is developed by the
[Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the
[Robotics Institute](https://www.ri.cmu.edu) at
[Carnegie Mellon University](http://www.cmu.edu).

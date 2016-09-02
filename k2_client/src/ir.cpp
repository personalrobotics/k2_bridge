/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -  Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
 -  Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the     distribution.
 -  Neither the name of Carnegie Mellon University nor the names of its contributors 
    may be used to endorse or promote products derived from this software without 
    specific prior written  permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************/
#include <boost/asio.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <yaml-cpp/yaml.h>

using boost::asio::ip::tcp;

constexpr size_t image_width = 512;
constexpr size_t image_height = 424;
constexpr size_t image_size = image_width * image_height * 2; // 16-bit IR image
constexpr size_t frame_size = image_size + sizeof(unsigned long); // image + timestamp 

unsigned char frame_buffer[frame_size];


int main(int argc, char *argv[])
{
    // Initialize this ROS node.
    ros::init(argc, argv, "k2_ir", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    // Retrieve the hostname and port of the k2_server.
    std::string server_host, server_port, frame_id;
    n.getParam("host", server_host);
    n.param<std::string>("port", server_port, "9002"); // default for k2_server IR
    n.param<std::string>("frame_id", frame_id, "/k2/depth_frame");

    // Create a Boost ASIO service to handle server connection.
    boost::asio::io_service io_service;

    // Get a list of endpoints corresponding to the server hostname and port.
    tcp::resolver resolver(io_service);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve({server_host, server_port});

    // Try each endpoint until we successfully establish a connection.
    tcp::socket socket(io_service);
    try
    {
        boost::asio::connect(socket, endpoint_iterator);
    }
    catch (boost::system::system_error const& e)
    {
        ROS_FATAL("Failed to connect to k2 server '%s:%s': %s",
                  server_host.c_str(), server_port.c_str(), e.what());
        return -1;
    }

    // Create a ROS publisher for the deserialized stream output.
    image_transport::ImageTransport image_transport(n);
    image_transport::CameraPublisher camera_publisher =
        image_transport.advertiseCamera("image", 1);
    camera_info_manager::CameraInfoManager camera_info_manager(n, "ir");
    camera_info_manager.loadCameraInfo("");
    cv::Mat image(cv::Size(image_width, image_height), CV_16UC1, frame_buffer);

    while(ros::ok())
    {
        // Read the next image from the server.
        boost::asio::read(socket, boost::asio::buffer(frame_buffer, frame_size));

        // Extract the timestamp (packed at end of image).
        unsigned long timestamp = *reinterpret_cast<unsigned long *>(&frame_buffer[image_size]);

        // Extract the image from the buffer
        cv::flip(image, image, 1);

        cv_bridge::CvImage cv_image;
        cv_image.header.frame_id = frame_id;
        cv_image.encoding = "mono16";
        cv_image.image = image;

        sensor_msgs::Image ros_image;
        cv_image.toImageMsg(ros_image);

        sensor_msgs::CameraInfo camera_info = camera_info_manager.getCameraInfo();
        camera_info.header.frame_id = cv_image.header.frame_id;

        // Send out the resulting message and request a new message.
        camera_publisher.publish(ros_image, camera_info, ros::Time(timestamp));
        boost::asio::write(socket, boost::asio::buffer("OK\n"));
        ros::spinOnce();
    }

    return 0;
}

/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Henny Admoni<hadmoni@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are 
permitted provided that the following conditions are met:

 -    Redistributions of source code must retain the above copyright notice, this list 
     of conditions and the following disclaimer.
 -    Redistributions in binary form must reproduce the above copyright notice, this 
     list of conditions and the following disclaimer in the documentation and/or other 
     materials provided with the     distribution.
 -    Neither the name of Carnegie Mellon University nor the names of its contributors 
     may be used to endorse or promote products derived from this software without 
     specific prior written     permission.
 
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
#include "k2_client/Face.h"

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <array>
#include <yaml-cpp/yaml.h>

using boost::asio::ip::tcp;


int main(int argc, char *argv[])
{
    // Initialize this ROS node.
    ros::init(argc, argv, "k2_face", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    // Retrieve the hostname and port of the k2_server.
    std::string server_host, server_port, frame_id;
    n.getParam("host", server_host);
    n.param<std::string>("port", server_port, "9005"); // default for k2_server faces
    n.param<std::string>("frame_id", frame_id, "/k2/depth_frame");

    // Create a Boost ASIO service to handle server connection.
    boost::asio::io_service io_service;

    // Create a Boost ASIO stream buffer to hold the unparsed input from the server.
    boost::asio::streambuf buffer;

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
    ros::Publisher facePublisher = n.advertise<k2_client::Face>("faces", 1);

    while(ros::ok())
    {
        // Read the next line from the server.
        boost::asio::read_until(socket, buffer, "\n");
        std::istream is(&buffer);
        std::string message;
        std::getline(is, message);

        // Parse the line from the server as JSON/YAML.
        const YAML::Node node = YAML::Load(message);
        if (!node)
        {
            ROS_WARN("Received malformed message '%s'.", message.c_str());
            continue;
        }

        const YAML::Node alignment = node["Alignment"];
        const YAML::Node animation_units = alignment["AnimationUnits"];

        // Convert the JSON message to a ROS message.
        k2_client::Face face;

        face.header.stamp =            ros::Time(node["Time"].as<unsigned long>());
        face.header.frame_id =         frame_id;
        face.trackingId =              node["TrackingId"].as<unsigned long>();

        face.jawOpen =                 animation_units["JawOpen"].as<double>();
        face.lipPucker =               animation_units["LipPucker"].as<double>();
        face.jawSlideRight =           animation_units["JawSlideRight"].as<double>();
        face.lipStretcherRight =       animation_units["LipStretcherRight"].as<double>();
        face.lipStretcherLeft =        animation_units["LipStretcherLeft"].as<double>();
        face.lipCornerPullerLeft =     animation_units["LipCornerPullerLeft"].as<double>();
        face.lipCornerPullerRight =    animation_units["LipCornerPullerRight"].as<double>();
        face.lipCornerDepressorLeft =  animation_units["LipCornerDepressorLeft"].as<double>();
        face.lipCornerDepressorRight = animation_units["LipCornerDepressorRight"].as<double>();
        face.leftCheekPuff =           animation_units["LeftcheekPuff"].as<double>();
        face.rightCheekPuff =          animation_units["RightcheekPuff"].as<double>();
        face.leftEyeClosed =           animation_units["LefteyeClosed"].as<double>();
        face.rightEyeClosed =          animation_units["RighteyeClosed"].as<double>();
        face.leftEyebrowLowerer =      animation_units["LefteyebrowLowerer"].as<double>();
        face.rightEyebrowLowerer =     animation_units["RighteyebrowLowerer"].as<double>();
        face.lowerLipDepressorLeft =   animation_units["LowerlipDepressorLeft"].as<double>();
        face.lowerLipDepressorRight =  animation_units["LowerlipDepressorRight"].as<double>();
        face.lowerLipDepressorLeft =   animation_units["LowerlipDepressorLeft"].as<double>();
        face.lowerLipDepressorLeft =   animation_units["LowerlipDepressorLeft"].as<double>();
        
        face.headPivotPoint.x =        alignment["HeadPivotPoint"]["X"].as<double>();
        face.headPivotPoint.y =        alignment["HeadPivotPoint"]["Y"].as<double>();
        face.headPivotPoint.z =        alignment["HeadPivotPoint"]["Z"].as<double>();

        face.faceOrientation.x =       alignment["FaceOrientation"]["X"].as<double>();
        face.faceOrientation.y =       alignment["FaceOrientation"]["Y"].as<double>();
        face.faceOrientation.z =       alignment["FaceOrientation"]["Z"].as<double>();
        face.faceOrientation.w =       alignment["FaceOrientation"]["W"].as<double>();

        // Send out the resulting message and request a new message.
        facePublisher.publish(face);
        boost::asio::write(socket, boost::asio::buffer("OK\n"));
        ros::spinOnce();
    }

    return 0;
}

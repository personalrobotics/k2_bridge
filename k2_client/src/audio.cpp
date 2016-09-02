/*************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia <ajakhoti@andrew.cmu.edu>
         Prasanna Velagapudi <pkv@cs.cmu.edu>

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
#include "k2_client/Audio.h"

#include <boost/asio.hpp>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

using boost::asio::ip::tcp;

int main(int argc, char *argv[])
{
    // Initialize this ROS node.
    ros::init(argc, argv, "k2_audio", ros::init_options::AnonymousName);
    ros::NodeHandle n("~");

    // Retrieve the hostname and port of the k2_server.
    std::string server_host, server_port, frame_id;
    n.getParam("host", server_host);
    n.param<std::string>("port", server_port, "9004"); // default for k2_server audio
    n.param<std::string>("frame_id", frame_id, "/k2/depth_frame");

    // Create a Boost ASIO service to handle server connection.
    boost::asio::io_service io_service;

    // Create a Boost ASIO stream buffer to hold the unparsed input from the server.
    boost::asio::streambuf buffer;

    // Get a list of endpoints corresponding to the server hostname and port.
    tcp::resolver resolver(io_service);
    tcp::resolver::query query(server_host, server_port);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

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
    ros::Publisher audioPublisher = n.advertise<k2_client::Audio>("audio", 1);

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

        // Convert the JSON message to a ROS message.
        k2_client::Audio audio;
        audio.header.stamp = ros::Time(node["utcTime"].as<unsigned long>());
        audio.header.frame_id = frame_id;
        audio.beamAngle = node["beamAngle"].as<double>();
        audio.beamAngleConfidence = node["beamAngleConfidence"].as<double>();
        for (int i=0; i<256; i++)
            audio.audioStream.push_back(node["audioStream"][i].as<float>());
        audio.numBytesPerSample = node["numBytesPerSample"].as<unsigned>();
        audio.numSamplesPerFrame = node["numSamplesPerFrame"].as<unsigned>();
        audio.frameLifeTime = node["frameLifeTime"].as<double>();
        audio.samplingFrequency = node["samplingFrequency"].as<unsigned>();

        // Send out the resulting message and request a new message.
        audioPublisher.publish(audio);
        boost::asio::write(socket, boost::asio::buffer("OK\n"));
        ros::spinOnce();
    }

    return 0;
}

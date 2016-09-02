k2_server
=========
Server application for Kinect for Windows v2

This software is a part of software package for integrating kinect for windows v2 with ROS. The software package is divided into two parts. One part runs on windows machine and dumps the data over the network, while, the other part runs on linux machine which reads the stream and publishes appropriate ROS topics.

This software is meant to be run on the windows side of the system. The corresponding package to run on linux side is called k2_client and can be found at: https://github.com/personalrobotics/k2_client

Setting up the software
=======================

The windows part of the setup can be done in two ways. You can either build from source or use pre-built binaries. You need to follow only one of following two sections to setup your windows machine. Building from source offers more flexibility to tweak the code, while using pre-built binaries lets you get started quickly without having to do install a lot of software.

1) Build from source
---------------------
A) Install windows as you would on a regular machine.

B) Download and install Microsoft Visual Studio Express 2013 for Windows Desktop from following link. Make sure to download the version for Windows desktop and not others.

MS Visual Studio: https://app.vssps.visualstudio.com/profile/review?download=true&slcid=0x409&context=eyJwZSI6MSwicGMiOjEsImljIjoxLCJhbyI6MSwiYW0iOjAsIm9wIjpudWxsLCJhZCI6bnVsbCwiZmEiOjAsImF1IjpudWxsLCJjdiI6OTQ0Njg4NjIzLCJmcyI6MCwic3UiOjAsImVyIjoxfQ2

C) Download and install the kinect SDK from following link.
Kinect SDK: http://www.microsoft.com/en-us/download/details.aspx?id=43661

D) Download the k2_server software from following page and extract it to any location of your choice.

k2_server source: https://github.com/personalrobotics/k2_server/tree/service

E) Connect the Kinect to the USB-3.0 port of the windows machine and start one of the sample applications to check if everything is working. If the sample don't run, refer to MSDN for troubleshooting tips. Once done, close all the kinect applications.

F) Disable IPv6 on the machine by following the instructions on following page.
DisableIPv6: http://www.techunboxed.com/2012/08/how-to-disable-ipv6-in-windows-8.html

G) Start Visual Studio Express 2013 and open the k2_server project that you downloaded in step no.4.

H) Select build option from the build menu to compile the code.

I) To run, simply click on start button or run the kinect2server.exe executable from the folder "Your_sofware_location \ k2_server \ kinect2server \ bin \ (appropriate folder based on build settings)"

2) Use pre-built binaries
-------------------------

A) Install windows as you would on a regular machine.

B) Download the binaries from following link and extract it to any location of you choice

k2_server binaries: https://github.com/personalrobotics/k2_server/tree/Binaries

C) Disable IPv6 on the machine by following the instructions on this page.

D) (optional) For convenience, create a desktop shortcut of kinect2server.exe file from the extracted folder.

E) Start the application. A kinect logo in the notification tray denotes that the application is active.

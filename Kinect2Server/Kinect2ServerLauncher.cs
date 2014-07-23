using System;
using System.ServiceProcess;

namespace PersonalRobotics.Kinect2Server
{
    static class Kinect2ServerLauncher
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main()
        {
            System.ServiceProcess.ServiceBase.Run(new Kinect2ServerService());
        }
    }
}

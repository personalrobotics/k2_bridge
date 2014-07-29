using System;
using System.ServiceProcess;
using System.Windows.Forms;

namespace PersonalRobotics.Kinect2Server
{
    static class Kinect2ServerLauncher
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        public static void Main()
        {
            if (Environment.UserInteractive)
            {
                Application.Run(new Kinect2ServerTray());
            }
            else
            {
                System.ServiceProcess.ServiceBase.Run(new Kinect2ServerService());
            }
        }
    }
}

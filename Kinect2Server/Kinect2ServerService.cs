using System;
using System.ServiceProcess;

namespace PersonalRobotics.Kinect2Server
{
    public class Kinect2ServerService : ServiceBase
    {
        public Kinect2ServerService()
        {
            this.ServiceName = "Kinect2Server";
            this.CanStop = true;
            this.CanPauseAndContinue = false;
            this.AutoLog = true;
        }

        protected override void OnStart(string[] args)
        {
            // TODO: Add startup stuff.
        }

        protected override void OnStop()
        {
            // TODO: Add shutdown stuff.
        }
    }
}

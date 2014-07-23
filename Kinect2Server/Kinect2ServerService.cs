using Microsoft.Kinect;
using System;
using System.Diagnostics;
using System.ServiceProcess;
using System.Windows.Media;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// A service that publishes data from the Kinect2 over TCP sockets.
    /// </summary>
    /// See: http://msdn.microsoft.com/en-us/library/system.serviceprocess.servicebase(v=vs.110).aspx
    /// 
    public class Kinect2ServerService : ServiceBase
    {
        private KinectSensor kinect;
        private MultiSourceFrameReader reader;

        private AsyncNetworkConnector colorConnector;
        private AsyncNetworkConnector depthConnector;
        private AsyncNetworkConnector irConnector;

        private byte[] colorArray;
        private ushort[] depthArray;
        private ushort[] irArray;
        private byte[] byteDepthArray;
        private byte[] byteIRArray;

        private static readonly int BYTES_PER_COLOR_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private const int BYTES_PER_DEPTH_PIXEL = 2;
        private const int BYTES_PER_IR_PIXEL = 2;

        public Kinect2ServerService()
        {
            this.ServiceName = "Kinect2Server";
            this.CanStop = true;
            this.CanPauseAndContinue = false;
            this.AutoLog = true;
        }

        protected override void OnStart(string[] args)
        {
            // Try to open the first available Kinect sensor.
            this.kinect = KinectSensor.Default;
            if (this.kinect == null)
            {
                EventLog.WriteEntry("No Kinect device was detected.");  
                ExitCode = -1;
                throw new KinectException("No kinect device was detected.");
            }
            else
            {
                this.kinect.Open();
            }

            // Register as a handler for the image data being returned by the Kinect.
            this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared);
            if (this.reader == null)
            {
                EventLog.WriteEntry("Unable to connect to Kinect data stream.");
                ExitCode = -2;
                throw new KinectException("Unable to connect to Kinect data stream.");
            }
            else
            {
                this.reader.MultiSourceFrameArrived += this.OnFrameArrived;
            }

            // Allocate storage for the data from the Kinect.
            this.colorArray = new byte[this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL];
            this.depthArray = new ushort[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];
            this.irArray = new ushort[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width];
            this.byteDepthArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL];
            this.byteIRArray = new byte[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL];

            // Create network connectors that will send out the data when it is received.
            this.colorConnector = new AsyncNetworkConnector(Properties.Settings.Default.RgbImagePort);
            this.depthConnector = new AsyncNetworkConnector(Properties.Settings.Default.DepthImagePort);
            this.irConnector = new AsyncNetworkConnector(Properties.Settings.Default.IrImagePort);

            // Open the server connections.
            this.colorConnector.Listen();
            this.depthConnector.Listen();
            this.irConnector.Listen();
        }

        protected override void OnStop()
        {
            this.kinect.Close();
            this.colorConnector.Close();
            this.depthConnector.Close();
            this.irConnector.Close();

            this.reader.Dispose(); // TODO: Is this actually necessary?
            this.kinect.Dispose();
            this.colorConnector.Dispose();
            this.depthConnector.Dispose();
            this.irConnector.Dispose();
        }

        private void OnFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            
            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyConvertedFrameDataToArray(this.colorArray, ColorImageFormat.Bgra);
                    this.colorConnector.Broadcast(this.colorArray);
                }
            }

            using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    depthFrame.CopyFrameDataToArray(this.depthArray);   //Ushort Array ! Use BitConverter.getBytes() to convert to two bytes per each uShort. it gives low byte followed by high byte
                    Buffer.BlockCopy(this.depthArray, 0, this.byteDepthArray, 0, this.byteDepthArray.Length);
                    this.depthConnector.Broadcast(this.byteDepthArray);
                }
            }

            using (InfraredFrame irFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
            {
                if (irFrame != null)
                {
                    irFrame.CopyFrameDataToArray(this.irArray);     //Ushort Array ! Use BitConverter.getBytes() to convert to two bytes per each uShort. it gives low byte followed by high byte
                    Buffer.BlockCopy(this.irArray, 0, this.byteIRArray, 0, this.byteIRArray.Length);
                    this.irConnector.Broadcast(this.byteIRArray);
                }
            }
        }
    }

    /// <summary>
    /// An exception indicating that a Kinect was not detected.
    /// </summary>
    public class KinectException : Exception
    {
        public KinectException()
        {
        }

        public KinectException(string message)
            : base(message)
        {
        }

        public KinectException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }
}

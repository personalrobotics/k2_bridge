/*******************************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are permitted provided 
that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list of conditions and 
    the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
    and the following disclaimer in the documentation and/or other materials provided with the 
    distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors may be used to 
    endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************************************/
using Microsoft.Kinect;
using System;
using System.Diagnostics;
using System.ServiceProcess;
using System.Windows.Media;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.IO;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// A service that publishes data from the Kinect2 over TCP sockets.
    /// </summary>
    /// See: http://msdn.microsoft.com/en-us/library/system.serviceprocess.servicebase(v=vs.110).aspx
    /// 
    public class Kinect2ServerService : ServiceBase
    {
        KinectSensor kinect;
        MultiSourceFrameReader reader;
        AudioSource audioSource;
        AudioBeamFrameReader audioReader;

        AsyncNetworkConnector colorConnector;
        AsyncNetworkConnector depthConnector;
        AsyncNetworkConnector irConnector;
        AsyncNetworkConnector bodyConnector;
        AsyncNetworkConnector audioConnector;
        AsyncNetworkConnector pointConnector;

        byte[] colorArray;
        ushort[] depthArray;
        ushort[] irArray;
        CameraSpacePoint[] pointArray;

        byte[] byteColorArray;
        byte[] byteDepthArray;
        byte[] byteIRArray;
        Body[] bodyArray;
        AudioContainer audioContainer;


        static readonly int BYTES_PER_COLOR_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        const int BYTES_PER_DEPTH_PIXEL = 2;
        const int BYTES_PER_IR_PIXEL = 2;
        const int BYTES_PER_3D_POINT = 3 * sizeof(float);

        public Kinect2ServerService()
        {
            this.ServiceName = "Kinect2Server";
            this.CanStop = true;
            this.CanPauseAndContinue = false;
            this.AutoLog = true;
        }

        /// <summary>
        /// Property that indicates whether the Kinect Server is connected to a sensor.
        /// </summary>
        public bool IsConnected { get { return (this.kinect != null) && kinect.IsAvailable; } }

        /// <summary>
        /// Event that triggers when the server detects a Kinect connection or disconnecting.
        /// </summary>
        public event EventHandler<IsConnectedChangedEventArgs> IsConnectedChanged;

        protected override void OnStart(string[] args)
        {
            // Try to open the first available Kinect sensor.
            this.kinect = KinectSensor.GetDefault();
            if (this.kinect == null)
            {
                EventLog.WriteEntry("No Kinect device was detected.");  
                ExitCode = -1;
                throw new KinectException("No kinect device was detected.");
            }
            else
            {
                this.kinect.Open();
                this.kinect.IsAvailableChanged += this.OnAvailableChanged;
            }

            // Register as a handler for the image data being returned by the Kinect.
            this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
            this.audioSource = this.kinect.AudioSource;
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
            if (this.audioSource == null)
            {
                EventLog.WriteEntry("Unable to open audio source on kinect");
                ExitCode = -3;
                throw new KinectException("Unable to connect to kinect audio source");
            }
            else
            {
                this.audioReader = this.audioSource.OpenReader();
                if (this.audioReader == null)
                    Console.WriteLine("Issues with audio reader");
                else
                    this.audioReader.FrameArrived += this.onAudioFrameArrived;
            }


            // Allocate storage for the data from the Kinect.
            this.colorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL)];
            this.depthArray = new ushort[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];
            this.irArray = new ushort[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width];
            this.pointArray = new CameraSpacePoint[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];

            this.byteColorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL) + sizeof(double)];
            this.byteDepthArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL + sizeof(double)];
            this.byteIRArray = new byte[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL + sizeof(double)];
            this.bodyArray = new Body[this.kinect.BodyFrameSource.BodyCount];

            this.audioContainer = new AudioContainer();
            this.audioContainer.samplingFrequency = 16000;
            this.audioContainer.frameLifeTime = 0.016;
            this.audioContainer.numSamplesPerFrame = (int)(this.audioContainer.samplingFrequency * this.audioContainer.frameLifeTime);
            this.audioContainer.numBytesPerSample = sizeof(float);
            this.audioContainer.audioStream = new float[256];
            
            // Create network connectors that will send out the data when it is received.
            this.colorConnector = new AsyncNetworkConnector(Properties.Settings.Default.RgbImagePort);
            this.depthConnector = new AsyncNetworkConnector(Properties.Settings.Default.DepthImagePort);
            this.irConnector = new AsyncNetworkConnector(Properties.Settings.Default.IrImagePort);
            this.bodyConnector = new AsyncNetworkConnector(Properties.Settings.Default.BodyPort);
            this.audioConnector = new AsyncNetworkConnector(Properties.Settings.Default.AudioPort);
            this.pointConnector = new AsyncNetworkConnector(Properties.Settings.Default.PointCloudPort);

            // Open the server connections.
            this.colorConnector.Listen();
            this.depthConnector.Listen();
            this.irConnector.Listen();
            this.bodyConnector.Listen();
            this.audioConnector.Listen();
            this.pointConnector.Listen();
            
        }

        protected override void OnStop()
        {
            this.kinect.Close();
            this.colorConnector.Close();
            this.depthConnector.Close();
            this.irConnector.Close();
            this.bodyConnector.Close();
            this.audioConnector.Close();
            this.pointConnector.Close();

            this.reader.Dispose(); // TODO: Is this actually necessary?
            this.audioReader.Dispose();
            this.colorConnector.Dispose();
            this.depthConnector.Dispose();
            this.irConnector.Dispose();
            this.bodyConnector.Dispose();
            this.audioConnector.Dispose();
            this.pointConnector.Dispose();
        }

        private void OnFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyConvertedFrameDataToArray(this.colorArray, ColorImageFormat.Bgra);
                    System.Buffer.BlockCopy(this.colorArray, 0, this.byteColorArray,0,(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL));
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteColorArray, (this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL), sizeof(double));
                    this.colorConnector.Broadcast(this.byteColorArray);
                }
            }

            using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    depthFrame.CopyFrameDataToArray(this.depthArray);
                    System.Buffer.BlockCopy(this.depthArray, 0, this.byteDepthArray, 0, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteDepthArray, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL,sizeof(double));
                    this.depthConnector.Broadcast(this.byteDepthArray);

                    // Generate point cloud from depth image
                    this.kinect.CoordinateMapper.MapDepthFrameToCameraSpace(this.depthArray, pointArray);

                    // Write the point cloud to a buffer
                    using (MemoryStream stream = new MemoryStream(this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_3D_POINT))
                    {
                        foreach (CameraSpacePoint csp in pointArray)
                        {
                            stream.Write(BitConverter.GetBytes(csp.X), 0, sizeof(float));
                            stream.Write(BitConverter.GetBytes(csp.Y), 0, sizeof(float));
                            stream.Write(BitConverter.GetBytes(csp.Z), 0, sizeof(float));
                        }
                        stream.Write(BitConverter.GetBytes(utcTime), 0, sizeof(double));
                        this.pointConnector.Broadcast(stream.ToArray());
                    }
                }
            }

            using (InfraredFrame irFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
            {
                if (irFrame != null)
                {
                    irFrame.CopyFrameDataToArray(this.irArray);
                    System.Buffer.BlockCopy(this.irArray, 0, this.byteIRArray, 0, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteIRArray, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL, sizeof(double));
                    this.irConnector.Broadcast(this.byteIRArray);
                }
            }

            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    bodyFrame.GetAndRefreshBodyData(this.bodyArray);
                    string jsonString = JsonConvert.SerializeObject(this.bodyArray);
                    int diff = 28000 - jsonString.Length;
                    for (int i = 0; i < diff;i++ )
                    {
                        jsonString += " ";
                    }
                    byte[] bodyByteArray = new byte[jsonString.Length*sizeof(char) + sizeof(double)];
                    System.Buffer.BlockCopy(jsonString.ToCharArray(), 0, bodyByteArray, 0, jsonString.Length * sizeof(char));
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, bodyByteArray, jsonString.Length * sizeof(char),sizeof(double));
                    this.bodyConnector.Broadcast(bodyByteArray);
                }
            }
        }

        private void onAudioFrameArrived(object sender,AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference audioFrameRefrence = e.FrameReference;
            try
            {
                AudioBeamFrameList frameList = audioFrameRefrence.AcquireBeamFrames();
                if (frameList != null)
                {
                    using (frameList)
                    {
                        IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                        foreach (AudioBeamSubFrame subFrame in subFrameList)
                        {
                            this.audioContainer.utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
                            this.audioContainer.beamAngle = subFrame.BeamAngle;
                            this.audioContainer.beamAngleConfidence = subFrame.BeamAngleConfidence;
                            byte[] array = new byte[this.audioSource.SubFrameLengthInBytes];
                            subFrame.CopyFrameDataToArray(array);
                            for (int i = 0; i < array.Length;i+=sizeof(float))
                            {
                                audioContainer.audioStream[(int)(i / sizeof(float))] = BitConverter.ToSingle(array, i);
                            }
                            string jsonString = JsonConvert.SerializeObject(this.audioContainer);
                            int diff = 4100 - jsonString.Length;
                            for (int i = 0; i < diff;i++)
                            {
                                jsonString += " ";
                            }
                            byte[] transmittedData = new byte[jsonString.Length*sizeof(char)];
                            System.Buffer.BlockCopy(jsonString.ToCharArray(), 0, transmittedData, 0, transmittedData.Length);
                            this.audioConnector.Broadcast(transmittedData);
                            subFrame.Dispose();
                        }
                    }
                    frameList.Dispose();
                }
            }
            catch
            {
            }
        }

        protected void OnAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.IsConnectedChanged(this, new IsConnectedChangedEventArgs(e.IsAvailable));
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

    /// <summary>
    /// Event triggered where the server connects or disconnects from a Kinect.
    /// </summary>
    public class IsConnectedChangedEventArgs : EventArgs
    {
        bool isConnected;
        public IsConnectedChangedEventArgs(bool isConnected)
        {
            this.isConnected = isConnected;
        }

        public bool IsConnected { get { return isConnected; } }
    }
}

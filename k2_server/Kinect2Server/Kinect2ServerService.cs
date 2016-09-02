﻿/*******************************************************************************************************
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
using Microsoft.Kinect.Face;
using System;
using System.Drawing;
using System.Collections.Generic;
using System.Diagnostics;
using System.ServiceProcess;
using System.Runtime.InteropServices;
using Newtonsoft.Json;

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
        HighDefinitionFaceFrameSource[] faceSources;
        HighDefinitionFaceFrameReader[] faceReaders;

        AsyncNetworkConnector colorConnector;
        AsyncNetworkConnector depthConnector;
        AsyncNetworkConnector irConnector;
        AsyncNetworkConnector bodyConnector;
        AsyncNetworkConnector audioConnector;
        AsyncNetworkConnector faceConnector;

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

            // Register as a handler for the audio source data being returned by the Kinect.
            this.audioSource = this.kinect.AudioSource;
            if (this.audioSource == null)
            {
                EventLog.WriteEntry("Unable to connect to Kinect audio source.");
                ExitCode = -3;
                throw new KinectException("Unable to connect to Kinect audio source.");
            }

            // Register as a handler for the audio reader data being returned by the Kinect.
            this.audioReader = this.audioSource.OpenReader();
            if (this.audioReader == null)
            {
                EventLog.WriteEntry("Unable to create reader for Kinect audio source.");
                ExitCode = -4;
                throw new KinectException("Unable to create reader for Kinect audio source.");
            }
            else
            {
                this.audioReader.FrameArrived += this.onAudioFrameArrived;
            }

            // Create an array of face sources/readers for each possible body.
            // These will be activated on demand as the corresponding bodies are tracked.
            this.faceSources = new HighDefinitionFaceFrameSource[this.kinect.BodyFrameSource.BodyCount];
            this.faceReaders = new HighDefinitionFaceFrameReader[this.kinect.BodyFrameSource.BodyCount];

            for (var i = 0; i < faceSources.Length; ++i)
            {
                // Register as a handler for the face source data being returned by the Kinect.
                this.faceSources[i] = new HighDefinitionFaceFrameSource(this.kinect);
                if (this.faceSources[i] == null)
                {
                    EventLog.WriteEntry("Unable to create Kinect face source [" + i + "].");
                    ExitCode = -5;
                    throw new KinectException("Unable to create Kinect face source [" + i + "].");
                }

                // Register as a handler for the face reader data being returned by the Kinect.
                this.faceReaders[i] = this.faceSources[i].OpenReader();
                if (this.faceReaders[i] == null)
                {
                    EventLog.WriteEntry("Unable to create reader for Kinect face source [" + i + "].");
                    ExitCode = -6;
                    throw new KinectException("Unable to create reader for Kinect face source [" + i + "].");
                }
                else
                {
                    this.faceReaders[i].FrameArrived += this.onFaceFrameArrived;
                }
            }


            // Create network connectors that will send out the data when it is received.
            this.colorConnector = new AsyncNetworkConnector(Properties.Settings.Default.RgbImagePort);
            this.depthConnector = new AsyncNetworkConnector(Properties.Settings.Default.DepthImagePort);
            this.irConnector = new AsyncNetworkConnector(Properties.Settings.Default.IrImagePort);
            this.bodyConnector = new AsyncNetworkConnector(Properties.Settings.Default.BodyPort);
            this.audioConnector = new AsyncNetworkConnector(Properties.Settings.Default.AudioPort);
            this.faceConnector = new AsyncNetworkConnector(Properties.Settings.Default.FacePort);

            // Open the server connections.
            this.colorConnector.Listen();
            this.depthConnector.Listen();
            this.irConnector.Listen();
            this.bodyConnector.Listen();
            this.audioConnector.Listen();
            this.faceConnector.Listen();
        }

        protected override void OnStop()
        {
            this.kinect.Close();
            this.colorConnector.Close();
            this.depthConnector.Close();
            this.irConnector.Close();
            this.bodyConnector.Close();
            this.audioConnector.Close();
            this.faceConnector.Close();

            this.reader.Dispose();
            this.audioReader.Dispose();
            this.colorConnector.Dispose();
            this.depthConnector.Dispose();
            this.irConnector.Dispose();
            this.bodyConnector.Dispose();
            this.audioConnector.Dispose();
            this.faceConnector.Dispose();
        }

        private void OnFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Acquire current Kinect frame reference.
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // Record the current Unix epoch timestamp and convert it to a byte array for serialization.
            long timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();
            byte[] timestampBytes = BitConverter.GetBytes(timestamp);

            // If clients exist, convert the RGB frame to a byte array and send it followed by a timestamp.
            if (this.colorConnector.HasClients)
            {
                using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
                {
                    if (colorFrame != null)
                    {
                        // Allocate a new byte buffer to store this RGB frame and timestamp.
                        var colorArraySize = colorFrame.ColorFrameSource.FrameDescription.Height *
                                             colorFrame.ColorFrameSource.FrameDescription.Width *
                                             3; // BGR8 = 24bpp
                        var colorBuffer = new byte[colorArraySize + sizeof(long)];
                        var colorRect = new Rectangle(0, 0,
                                                      colorFrame.ColorFrameSource.FrameDescription.Width,
                                                      colorFrame.ColorFrameSource.FrameDescription.Height);

                        // Wrap RGB frames into bitmap buffers.
                        var bmp32 = new Bitmap(colorFrame.ColorFrameSource.FrameDescription.Width,
                                               colorFrame.ColorFrameSource.FrameDescription.Height,
                                               System.Drawing.Imaging.PixelFormat.Format32bppRgb);
                        var bmp24 = new Bitmap(bmp32.Width, bmp32.Height, bmp32.Width * 3,
                                               System.Drawing.Imaging.PixelFormat.Format24bppRgb,
                                               Marshal.UnsafeAddrOfPinnedArrayElement(colorBuffer, 0));

                        // Lock the bitmap's bits.
                        System.Drawing.Imaging.BitmapData bmpData =
                            bmp32.LockBits(colorRect, System.Drawing.Imaging.ImageLockMode.ReadWrite, bmp32.PixelFormat);
                        IntPtr bmpPtr = bmpData.Scan0;
                        colorFrame.CopyConvertedFrameDataToIntPtr(bmpPtr, (uint)(bmpData.Width * bmpData.Height * 4), ColorImageFormat.Bgra);
                        bmp32.UnlockBits(bmpData);

                        // Convert from 32bpp to 24bpp using System.Drawing.
                        using (Graphics gr = Graphics.FromImage(bmp24))
                        {
                            gr.DrawImage(bmp32, new Rectangle(0, 0, bmp24.Width, bmp24.Height));
                        }

                        // Append the system timestamp to the end of the buffer.
                        Buffer.BlockCopy(timestampBytes, 0, colorBuffer, (int)colorArraySize, sizeof(long));

                        // Transmit the byte buffer to color clients.
                        this.colorConnector.Broadcast(colorBuffer);
                    }
                }
            }

            // If clients exist, convert the RGB frame to a byte array and send it followed by a timestamp.
            if (this.depthConnector.HasClients)
            {
                using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                {
                    if (depthFrame != null)
                    {
                        // Allocate a new byte buffer to store this depth frame and timestamp.
                        var depthArraySize = depthFrame.DepthFrameSource.FrameDescription.Height *
                                             depthFrame.DepthFrameSource.FrameDescription.Width *
                                             depthFrame.DepthFrameSource.FrameDescription.BytesPerPixel;
                        var depthBuffer = new byte[depthArraySize + sizeof(long)];

                        // Convert the depth frame into the byte buffer.
                        using (var depthFrameBuffer = depthFrame.LockImageBuffer())
                        {
                            Marshal.Copy(depthFrameBuffer.UnderlyingBuffer, depthBuffer, 0, (int)depthFrameBuffer.Size);
                        }

                        // Append the system timestamp to the end of the buffer.
                        Buffer.BlockCopy(timestampBytes, 0, depthBuffer, (int)depthArraySize, sizeof(long));

                        // Transmit the byte buffer to color clients.
                        this.depthConnector.Broadcast(depthBuffer);
                    }
                }
            }

            // If clients exist, convert the IR frame to a byte array and send it followed by a timestamp.
            if (this.irConnector.HasClients)
            {
                using (InfraredFrame irFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
                {
                    if (irFrame != null)
                    {
                        // Allocate a new byte buffer to store this IR frame and timestamp.
                        var irArraySize = irFrame.InfraredFrameSource.FrameDescription.Height *
                                          irFrame.InfraredFrameSource.FrameDescription.Width *
                                          irFrame.InfraredFrameSource.FrameDescription.BytesPerPixel;
                        var irBuffer = new byte[irArraySize + sizeof(long)];

                        // Convert the IR frame into the byte buffer.
                        using (var irFrameBuffer = irFrame.LockImageBuffer())
                        {
                            Marshal.Copy(irFrameBuffer.UnderlyingBuffer, irBuffer, 0, (int)irFrameBuffer.Size);
                        }

                        // Append the system timestamp to the end of the buffer.
                        Buffer.BlockCopy(timestampBytes, 0, irBuffer, (int)irArraySize, sizeof(long));

                        // Transmit the byte buffer to color clients.
                        this.irConnector.Broadcast(irBuffer);
                    }
                }
            }

            // If clients exist, convert the tracked skeletons to a JSON array and send it with a timestamp.
            if (this.bodyConnector.HasClients || this.faceConnector.HasClients)
            {
                using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
                {
                    if (bodyFrame != null)
                    {
                        var bodyArray = new Body[this.kinect.BodyFrameSource.BodyCount];
                        bodyFrame.GetAndRefreshBodyData(bodyArray);

                        // Configure tracking IDs for bodies that have been added.
                        if (this.faceConnector.HasClients)
                        {
                            for (var i = 0; i < bodyArray.Length; ++i)
                            {
                                // Only process the actively tracked bodies.
                                Body body = bodyArray[i];
                                if (!body.IsTracked) continue;

                                // Activate the corresponding face tracker using this body's tracking ID.
                                faceSources[i].TrackingId = body.TrackingId;
                            }
                        }

                        // Serialize body tracking information to clients.
                        if (this.bodyConnector.HasClients)
                        {
                            // Iterate through the full list of bodies (which might not all be tracked).
                            List<Body> bodyList = new List<Body>();
                            for(var i = 0; i < bodyArray.Length; ++i)
                            {
                                // Only process the actively tracked bodies.
                                Body body = bodyArray[i];
                                if (!body.IsTracked) continue;

                                // Add this body to the list of bodies that are serialized to clients.
                                bodyList.Add(body);
                            }

                            // Combine the body array with a timestamp.
                            // TODO: suppress invalid face tracking information!
                            Dictionary<string, object> bodyJson = new Dictionary<string, object>{
                                {"Time", timestamp},
                                {"Bodies", bodyList}
                            };

                            string json = JsonConvert.SerializeObject(bodyJson,
                                new JsonSerializerSettings { ContractResolver = new BodyContractResolver() }) + "\n";
                            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(json);
                            this.bodyConnector.Broadcast(bytes);
                        }
                    }
                }
            }
        }

        private void onAudioFrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            // Return if there are no audio clients.
            if (!this.audioConnector.HasClients) return;

            // Create an audio container representing Kinect audio buffer data.
            var audioContainer = new AudioContainer();
            audioContainer.samplingFrequency = 16000;
            audioContainer.frameLifeTime = 0.016;
            audioContainer.numSamplesPerFrame = (int)(audioContainer.samplingFrequency * audioContainer.frameLifeTime);
            audioContainer.numBytesPerSample = sizeof(float);
            audioContainer.audioStream = new float[256];

            // Record the current Unix epoch timestamp.
            audioContainer.timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();

            // TODO: add relative timestamp to audio?
            // this.audioContainer.relativeTime = e.FrameReference.RelativeTime.TotalMilliseconds;

            // Retrieve audio beams for current frame.
            AudioBeamFrameList frameList = e.FrameReference.AcquireBeamFrames();
            if (frameList == null) return;

            // Serialize all of the subframes and send as a JSON message.
            using (frameList)
            {
                // Only one audio beam is supported. Get the subframe list for the one beam.
                IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                // Consolidate the beam subframes into a single JSON message.
                foreach (AudioBeamSubFrame subFrame in subFrameList)
                {
                    using (subFrame)
                    {
                        audioContainer.beamAngle = subFrame.BeamAngle;
                        audioContainer.beamAngleConfidence = subFrame.BeamAngleConfidence;

                        byte[] array = new byte[subFrame.FrameLengthInBytes];
                        subFrame.CopyFrameDataToArray(array);
                        for (int i = 0; i < array.Length; i += sizeof(float))
                        {
                            audioContainer.audioStream[(int)(i / sizeof(float))] = BitConverter.ToSingle(array, i);
                        }

                        // Send audio data to clients.
                        string json = JsonConvert.SerializeObject(audioContainer,
                            new JsonSerializerSettings { ContractResolver = new AudioContractResolver() }) + "\n";
                        byte[] bytes = System.Text.Encoding.ASCII.GetBytes(json);
                        this.audioConnector.Broadcast(bytes);
                    }
                }
            }
        }

        private void onFaceFrameArrived(object sender, HighDefinitionFaceFrameArrivedEventArgs e)
        {
            // Return if there are no face clients.
            if (!this.faceConnector.HasClients) return;

            // Retrieve face data for current frame.
            var frame = e.FrameReference.AcquireFrame();
            if (frame == null) return;

            using (frame)
            {
                // Ignore untracked faces.
                if (!frame.IsTrackingIdValid) return;
                if (!frame.IsFaceTracked) return;

                // Record the current Unix epoch timestamp and convert it to a byte array for serialization.
                long timestamp = DateTimeOffset.Now.ToUnixTimeMilliseconds();

                // Retrieve face alignment data.
                var faceAlignment = new FaceAlignment();
                frame.GetAndRefreshFaceAlignmentResult(faceAlignment);

                // Combine the body array with a timestamp.
                Dictionary<string, object> faceJson = new Dictionary<string, object>{
                    {"Time", timestamp},
                    {"TrackingId", frame.HighDefinitionFaceFrameSource.TrackingId},
                    {"Alignment", faceAlignment},
                };

                // Send face data to clients.
                string json = JsonConvert.SerializeObject(faceJson,
                    new JsonSerializerSettings { ContractResolver = new FaceContractResolver() }) + "\n";
                byte[] bytes = System.Text.Encoding.ASCII.GetBytes(json);
                this.faceConnector.Broadcast(bytes);
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
    [Serializable]
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

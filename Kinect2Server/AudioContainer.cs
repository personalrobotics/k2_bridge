using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;

namespace PersonalRobotics.Kinect2Server
{
    struct AudioContainer
    {
        public float beamAngle;
        public float beamAngleConfidence;
        public byte[] audioStream;
        public int numBytesPerSample;
        public int numSamplesPerFrame;
        public double frameLifeTime;
        public int samplingFrequency;
    }
}

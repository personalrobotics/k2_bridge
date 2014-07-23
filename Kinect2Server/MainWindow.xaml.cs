using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;

namespace Kinect2Server
{
    public class FlaggedSocket
    {
        public Socket socket;
        public bool prevSendDoneFlag;

        public FlaggedSocket()
        {
            this.prevSendDoneFlag = true;
        }
    };
    public class AsyncNetworkConnectorServer
    {
        public IPAddress selfIPaddress = null;
        public IPEndPoint selfEndPoint = null;
        public int selfPortNumber;
        public string localPCName;

        public Socket listenerSocket = null;
        public SynchronizedCollection<FlaggedSocket> connectedClientList = null;
        public bool connectedToAtleastOne = false;

        public AsyncNetworkConnectorServer(int selfPortNum)
        {
            this.listenerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            this.selfPortNumber = selfPortNum;
            this.finishSocketGroundWork();
        }

        public void finishSocketGroundWork()
        {
            this.localPCName = System.Environment.MachineName;
            IPHostEntry selfInfo = Dns.GetHostEntry(this.localPCName);
            this.selfIPaddress = selfInfo.AddressList[0];
            this.selfEndPoint = new IPEndPoint(this.selfIPaddress, this.selfPortNumber);
            this.connectedClientList = new SynchronizedCollection<FlaggedSocket>();
            this.listenerSocket.Bind(this.selfEndPoint);
        }

        public void startListening()
        {
            this.listenerSocket.Listen(100);
            this.listenerSocket.BeginAccept(new AsyncCallback(startListeningCallBack), this);
        }

        public void startListeningCallBack(IAsyncResult ar)
        {
            try
            {
                AsyncNetworkConnectorServer connector = (AsyncNetworkConnectorServer)ar.AsyncState;
                FlaggedSocket tempSocket = new FlaggedSocket();
                tempSocket.socket = (connector.listenerSocket.EndAccept(ar));
                connector.connectedClientList.Add(tempSocket);
                Console.WriteLine("Added socket to client list");
                connector.listenerSocket.BeginAccept(new AsyncCallback(startListeningCallBack), connector);
            }
            catch (Exception e)
            {
                Console.WriteLine("Error in startListeningCallBack: " + e.ToString());
            }
        }

        public void recieveCallBack(IAsyncResult ar)
        {

        }

        public void sendCallBack(IAsyncResult ar)
        {
            try
            {
                FlaggedSocket tempSocket = (FlaggedSocket)ar.AsyncState;
                if (tempSocket.socket.Connected)
                {
                    int sentBytes = tempSocket.socket.EndSend(ar);
                    Console.WriteLine("Number of Bytes sent = " + sentBytes.ToString());
                    tempSocket.prevSendDoneFlag = true;
                }
            }
            catch (Exception e)
            {
                FlaggedSocket tempSocket = (FlaggedSocket)ar.AsyncState;
                Console.WriteLine("Error in sendCallBack: " + e.ToString());
                tempSocket.socket.Close();
                tempSocket.socket.Dispose();
                this.connectedClientList.Remove(tempSocket);
            }
        }

        public void receive()
        {

        }

        public void send(FlaggedSocket tempSocket, byte[] data)
        {
            try
            {
                if (tempSocket.prevSendDoneFlag)
                {
                    tempSocket.prevSendDoneFlag = false;
                    tempSocket.socket.BeginSend(data, 0, data.Length, 0, new AsyncCallback(this.sendCallBack), tempSocket);
                }
                else
                {
                    Console.WriteLine("Skipping");
                }

            }
            catch (Exception e)
            {
                Console.WriteLine("Error in send: " + e.ToString());
                tempSocket.socket.Close();
                tempSocket.socket.Dispose();
                this.connectedClientList.Remove(tempSocket);
            }
        }

        public void sendToAll(byte[] data)
        {
            IEnumerator<FlaggedSocket> enumerator = this.connectedClientList.GetEnumerator();
            try
            {
                while (enumerator.MoveNext())
                {
                    this.send(enumerator.Current, data);
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Error in sentToAll: " + e.ToString());
            }
        }

        public void closeSocket()
        {
            IEnumerator<FlaggedSocket> enumerator = this.connectedClientList.GetEnumerator();
            while (enumerator.MoveNext())
            {
                try
                {
                    enumerator.Current.socket.Shutdown(SocketShutdown.Both);
                    enumerator.Current.socket.Close();
                }
                catch (Exception e)
                {
                    Console.WriteLine("Error in closing: " + e.ToString());
                }
            }
        }
    };

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect = null;
        private Stopwatch stopwatch = null;
        private MultiSourceFrameReader reader = null;

        private byte[] colourArray = null;
        private ushort[] depthArray = null;
        private ushort[] IRArray = null;
        private byte[] byteDepthArray = null;
        private byte[] byteIRArray = null;

        private double numFramesPassed = 0;
        private int deltaTimeForFPS = 1;//In seconds
        private DateTime updateFPSMilestone = DateTime.Now;

        private readonly int bytesPerColorPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        private readonly int bytesPerDepthPixel = 2;
        private readonly int bytesPerIRPixel = 2;

        //TCP/IP crap
        private AsyncNetworkConnectorServer colorConnector = null;
        private AsyncNetworkConnectorServer depthConnector = null;
        private AsyncNetworkConnectorServer IRConnector = null;

        private const int BUFFER_SIZE_COLOR = 8294400;
        private const int BUFFER_SIZE_DEPTH = 4147200;
        private const int BUFFER_SIZE_IR = 4147200;

        private const int colorPort = 9000;
        private const int depthPort = 18000;
        private const int IRPort = 27000;

        private const string hostName = "herb2";
        //End of TCP/IP crap

        //Not Needed
        public WriteableBitmap colorBitMap;
        //private WriteableBitmap depthBitMap;
        //private WriteableBitmap IRBitMap;
        //End of not needed

        public MainWindow()
        {
            this.InitializeComponent();
            this.stopwatch = new Stopwatch();
            this.stopwatch.Start();
            this.kinect = KinectSensor.Default;
            if (this.kinect != null)
            {
                this.kinect.Open();

                this.colourArray = new byte[this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * this.bytesPerColorPixel];
                this.depthArray = new ushort[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];
                this.IRArray = new ushort[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width];
                this.byteDepthArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * this.bytesPerDepthPixel];
                this.byteIRArray = new byte[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * this.bytesPerIRPixel];

                this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared);
                this.reader.MultiSourceFrameArrived += this.frameArrivedCallback;
                this.updateFPSMilestone = DateTime.Now + TimeSpan.FromSeconds(this.deltaTimeForFPS);

                this.colorBitMap = new WriteableBitmap(this.kinect.ColorFrameSource.FrameDescription.Width,
                                                        this.kinect.ColorFrameSource.FrameDescription.Height,
                                                        96.0, 96.0, PixelFormats.Bgr32, null);

                //this.depthBitMap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
                //this.IRBitMap = new WriteableBitmap(IRFrameDescription.Width, IRFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            //instantiate sockets and get ip addresses. 
            this.colorConnector = new AsyncNetworkConnectorServer(colorPort);
            this.depthConnector = new AsyncNetworkConnectorServer(depthPort);
            this.IRConnector = new AsyncNetworkConnectorServer(IRPort);

            this.colorIPBox.Text = this.colorConnector.selfEndPoint.ToString();
            this.depthIPBox.Text = this.depthConnector.selfEndPoint.ToString();
            this.IRipBox.Text = this.IRConnector.selfEndPoint.ToString();
            //Create the connections
            this.colorConnector.startListening();
            this.depthConnector.startListening();
            this.IRConnector.startListening();
        }

        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.reader != null)
            {
                this.reader.Dispose();
                this.reader = null;
            }
            if (this.reader != null)
            {
                this.kinect.Close();
                this.kinect = null;
            }
            this.colorConnector.closeSocket();
            this.depthConnector.closeSocket();
            this.IRConnector.closeSocket();

        }

        public ImageSource dispColor
        {
            get
            {
                return this.colorBitMap;
            }
        }

        private void frameArrivedCallback(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            this.numFramesPassed++;
            if (DateTime.Now > updateFPSMilestone)
            {
                this.stopwatch.Stop();
                double fps = this.numFramesPassed / this.stopwatch.Elapsed.TotalSeconds;
                this.stopwatch.Reset();
                this.stopwatch.Start();
                this.updateFPSMilestone = DateTime.Now + TimeSpan.FromSeconds(this.deltaTimeForFPS);
                this.numFramesPassed = 0;
                this.statusBox.Text = fps.ToString();
            }
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            using (ColorFrame cFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (cFrame != null)
                {
                    cFrame.CopyConvertedFrameDataToArray(this.colourArray, ColorImageFormat.Bgra);
                    //Not Needed
                    this.colorBitMap.WritePixels(new Int32Rect(0, 0, cFrame.FrameDescription.Width, cFrame.FrameDescription.Height),
                                                    this.colourArray,
                                                    cFrame.FrameDescription.Width * this.bytesPerColorPixel,
                                                    0);
                    colorOutput.Source = this.colorBitMap;
                    colorConnector.sendToAll(this.colourArray);
                }
            }
            using (DepthFrame dFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (dFrame != null)
                {
                    dFrame.CopyFrameDataToArray(this.depthArray);   //Ushort Array ! Use BitConverter.getBytes() to convert to two bytes per each uShort. it gives low byte followed by high byte
                    Buffer.BlockCopy(this.depthArray, 0, this.byteDepthArray, 0, this.byteDepthArray.Length);
                    this.depthConnector.sendToAll(this.byteDepthArray);
                }
            }
            using (InfraredFrame IRFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
            {
                if (IRFrame != null)
                {
                    IRFrame.CopyFrameDataToArray(this.IRArray);     //Ushort Array ! Use BitConverter.getBytes() to convert to two bytes per each uShort. it gives low byte followed by high byte
                    Buffer.BlockCopy(this.IRArray, 0, this.byteIRArray, 0, this.byteIRArray.Length);
                    this.IRConnector.sendToAll(this.byteIRArray);
                }
            }
        }

        private void statusBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }
    }
}
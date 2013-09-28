using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.IO.Ports;
using Microsoft.Kinect;
using Coding4Fun.Kinect.Wpf;
using System.Drawing;
using System.Threading;
using System.Windows.Threading;
using System.Net;
using System.Net.Sockets;

namespace SkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        #region variables
        //Serial 
        SerialPort serial = new SerialPort();
        string recieved_data;
        Socket client;
        private byte[] sockSrvRecvBuff = new byte[50];		// Receive data buffer
        #endregion


        private int a = 130;

        
        public MainWindow()
        {
            InitializeComponent();

            //kinectSensorChooser1.Kinect.ElevationAngle = 0;
            socketServerStart();
        }

        bool closing = false;
        const int skeletonCount = 6; 
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];
        int humanTrackedIndex = 0;
        bool humanPreviouslyTracked = false;

        private void Connect_Comms(object sender, RoutedEventArgs e)
        {
            if (Connect_btn.Content == "Connect")
            {
                //Sets up serial port
                serial.PortName = Comm_Port_Names.Text;
                serial.BaudRate = Convert.ToInt32(Baud_Rates.Text);
                serial.Handshake = System.IO.Ports.Handshake.None;
                serial.Parity = Parity.None;
                serial.DataBits = 8;
                serial.StopBits = StopBits.One;
                serial.ReadTimeout = 200;
                serial.WriteTimeout = 50;
                serial.Open();

                //Sets button State and Creates function call on data recieved
                Connect_btn.Content = "Disconnect";
                serial.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(Recieve);

            }
            else
            {
                try // just in case serial port is not open could also be acheved using if(serial.IsOpen)
                {
                    serial.Close();
                    Connect_btn.Content = "Connect";
                }
                catch
                {
                }
            }
        }

        #region Recieving

        private delegate void UpdateUiTextDelegate(string text);
        private void Recieve(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            // Collecting the characters received to our 'buffer' (string).
            recieved_data = serial.ReadExisting();
            Dispatcher.Invoke(DispatcherPriority.Send, new UpdateUiTextDelegate(WriteData), recieved_data);
        }
        private void WriteData(string text)
        {
            TextSerialReceived.AppendText(text);
        }

        #endregion

        #region Sending

        private void Send_Data(object sender, RoutedEventArgs e)
        {
            SerialCmdSend(SerialData.Text);
            SerialData.Text = "";
        }
        public void SerialCmdSend(string data)
        {
            if (serial.IsOpen)
            {
                try
                {
                   
                    serial.Write(data);
                }
                catch (Exception ex)
                {
                    TextSerialReceived.AppendText("Failed to SEND" + data + "\n" + ex + "\n");
                }
            }
            
        }

        #endregion

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
            textBox0.AppendText("Window_Loaded\n");
        }

        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor old = (KinectSensor)e.OldValue;

            StopKinect(old);

            KinectSensor sensor = (KinectSensor)e.NewValue;

            if (sensor == null)
            {
                return;
            }

            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            //sensor.SkeletonStream.Enable(parameters);

            sensor.SkeletonStream.Enable();

            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30); 
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);

            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }

        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)
            {
                return;
            }

            //Get a skeleton
            Skeleton first =  GetFirstSkeleton(e);

            if (first == null)
            {
                return; 
            }

            //set scaled position
            //ScalePosition(headImage, first.Joints[JointType.Head]);
            //ScalePosition(leftEllipse, first.Joints[JointType.HandLeft]);
            //ScalePosition(rightEllipse, first.Joints[JointType.HandRight]);

            GetCameraPoint(first, e);

            float SpineX = first.Joints[JointType.Spine].Position.X;
            float SpineY = first.Joints[JointType.Spine].Position.Y;
            float SpineZ = first.Joints[JointType.Spine].Position.Z;
            TextSpineX.Text = SpineX.ToString();
            TextSpineY.Text = SpineY.ToString();
            TextSpineZ.Text = SpineZ.ToString();


            //判断x是否超出阈值
            if (SpineX>0.7&& SpineZ>0.4)//！！！还要判断一下z的变化量，但不知道如何求这个量
            {
                //MessageBox.Show("左转");
                
                serial.Write("#！3@"+(SpineZ-0.4)+"*");
            }
            else if (SpineX<-0.7 && SpineZ>0.4)
            {
                //MessageBox.Show("右转");
                serial.Write("#！4@" + (SpineZ - 0.4) + "*");
            }
            xelse if (SpineZ>1.2 &&SpineX<0.1 || SpineX>-0.1 )
            {
                //MessageBox.Show("要前进");
                serial.Write("#！1@" + (SpineZ - 0.4) + "*");
            }
            else if (SpineZ<0.4)
            {
                //MessageBox.Show("要停止");
                serial.Write("#！2@" + (SpineZ - 0.4) + "*");
            }
            // float SpineXBig = SpineX * 1000;

           
           // SerialCmdSend(SpineXBig.ToString("0000.")+'x');
            
            //SerialCmdSend(SpineZBig.ToString("0000.") + 'z');
        }

        void GetCameraPoint(Skeleton first, AllFramesReadyEventArgs e)
        {

            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null ||
                    kinectSensorChooser1.Kinect == null)
                {
                    return;
                }
                

                //Map a joint location to a point on the depth map
                //head
                DepthImagePoint headDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.Head].Position);
                //left hand
                DepthImagePoint leftDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position);
                //right hand
                DepthImagePoint rightDepthPoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position);


                //Map a depth point to a point on the color image
                //head
                ColorImagePoint headColorPoint =
                    depth.MapToColorImagePoint(headDepthPoint.X, headDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //left hand
                ColorImagePoint leftColorPoint =
                    depth.MapToColorImagePoint(leftDepthPoint.X, leftDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);
                //right hand
                ColorImagePoint rightColorPoint =
                    depth.MapToColorImagePoint(rightDepthPoint.X, rightDepthPoint.Y,
                    ColorImageFormat.RgbResolution640x480Fps30);

                //Set location
                CameraPosition(headImage, headColorPoint);
                CameraPosition(leftEllipse, leftColorPoint);
                CameraPosition(rightEllipse, rightColorPoint);
            }        
        }

        Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null; 
                }

                
                skeletonFrameData.CopySkeletonDataTo(allSkeletons);

                //get the first tracked skeleton
                Skeleton first;
                if (allSkeletons[humanTrackedIndex].TrackingState != SkeletonTrackingState.Tracked)
                {
                    if (humanPreviouslyTracked)
                    {
                        printDebug("Human Lost, index: " + humanTrackedIndex.ToString() + "\n");
                        humanPreviouslyTracked = false;
                    }
                    for (int i = 0; i < 6; i++)
                    {
                        if (allSkeletons[i].TrackingState == SkeletonTrackingState.Tracked)
                        {
                            humanTrackedIndex = i;
                            humanPreviouslyTracked = true;
                            printDebug("Human found, index: " + humanTrackedIndex.ToString() + "\n");
                            break;
                        }
                    }

                }
                first = allSkeletons[humanTrackedIndex];

                return first;

            }
        }

        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();

                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }



                }
            }
        }

        private void CameraPosition(FrameworkElement element, ColorImagePoint point)
        {
            //Divide by 2 for width and height so point is right in the middle 
            // instead of in top/left corner
            Canvas.SetLeft(element, point.X - element.Width / 2);
            Canvas.SetTop(element, point.Y - element.Height / 2);

        }

        private void ScalePosition(FrameworkElement element, Joint joint)
        {
            //convert the value to X/Y
            //Joint scaledJoint = joint.ScaleTo(1280, 720); 
            
            //convert & scale (.3 = means 1/3 of joint distance)
            Joint scaledJoint = joint.ScaleTo(1280, 720, .3f, .3f);

            Canvas.SetLeft(element, scaledJoint.Position.X);
            Canvas.SetTop(element, scaledJoint.Position.Y); 
            
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            closing = true; 
            StopKinect(kinectSensorChooser1.Kinect); 
        }

        private void textBox0_TextChanged(object sender, TextChangedEventArgs e)
        {
            textBox0.ScrollToEnd();
        }

        private void TextSerialReceived_Changed(object sender, TextChangedEventArgs e)
        {
            //TextSerialReceived.SelectionStart = TextSerialReceived.Text.Length;
            TextSerialReceived.ScrollToEnd();
        }

        private void socketServerStart()
        {
            IPAddress[] aryLocalAddr = null;
            string strHostName = "";
            const int nPortListen = 50000;
            try
            {
                // NOTE: DNS lookups are nice and all but quite time consuming.
                strHostName = Dns.GetHostName();
                IPHostEntry ipEntry = Dns.GetHostByName(strHostName);
                aryLocalAddr = ipEntry.AddressList;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error trying to get local address {0} ", ex.Message);
                printDebug("Error trying to get local address " + ex.Message + "\n");
            }

            // Verify we got an IP address. Tell the user if we did
            if (aryLocalAddr == null || aryLocalAddr.Length < 1)
            {
                Console.WriteLine("Unable to get local address");
                printDebug("Unable to get local address\n");
                return;
            }
            //Console.WriteLine("Listening on : [{0}] {1}", strHostName, aryLocalAddr[0]);
            printDebug("Listening on [" + strHostName + "] " + aryLocalAddr[0] +":" + nPortListen.ToString() + "\n");

            // Create the listener socket in this machines IP address
            Socket listener = new Socket(AddressFamily.InterNetwork,
                              SocketType.Stream, ProtocolType.Tcp);
            listener.Bind(new IPEndPoint(aryLocalAddr[0], nPortListen));
            //listener.Bind( new IPEndPoint( IPAddress.Loopback, 399 ) );
            // For use with localhost 127.0.0.1
            listener.Listen(10);

            // Setup a callback to be notified of connection requests
            listener.BeginAccept(new AsyncCallback(OnConnectRequest), listener);

            textBox0.AppendText("Begin accepting connection\n");
        }

        public void OnConnectRequest(IAsyncResult ar)
        {
            Socket listener = (Socket)ar.AsyncState;
            client = listener.EndAccept(ar);
            Console.WriteLine("Client {0}, joined", client.RemoteEndPoint);
            printDebug("Client ");
            printDebug(client.RemoteEndPoint.ToString());
            printDebug(" joined\n");

            listener.BeginAccept(new AsyncCallback(OnConnectRequest), listener);
            SetupReceiveCallback();
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            string strDataLine = sockSrvTxtSend.Text.ToString();
            // Convert to byte array and send.
            Byte[] byteDateLine =
                System.Text.Encoding.ASCII.GetBytes(strDataLine.ToCharArray());
            client.Send(byteDateLine, byteDateLine.Length, 0);
        }

        public void SetupReceiveCallback()
        {
            try
            {
                AsyncCallback receiveData = new AsyncCallback(OnReceivedData);
                client.BeginReceive(sockSrvRecvBuff, 0, sockSrvRecvBuff.Length, SocketFlags.None, receiveData, client);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Recieve callback setup failed! {0}", ex.Message);
                printDebug("Recieve callback setup failed!" + ex.Message + "\n");
            }
        }

        public void OnReceivedData(IAsyncResult ar)
        {
            //Socket client = (Socket)ar.AsyncState;
            Socket clienttmp = (Socket)ar.AsyncState;
            byte[] aryRet = GetReceivedData(ar);

            // If no data was recieved then the connection is probably dead
            if (aryRet.Length < 1)
            {
                Console.WriteLine("Client {0}, disconnected", clienttmp.RemoteEndPoint);
                printDebug("Client " + clienttmp.RemoteEndPoint.ToString() + " disconnected\n");
                clienttmp.Close();
                return;
            }
            printDebug("Received String:");
            string debugOutput = System.Text.Encoding.UTF8.GetString(aryRet);
            printDebug(debugOutput+ "\n");
            printDebug("Received Numbers:");
            for (int i = 0; i < aryRet.Length; i++)
            {
                printDebug(aryRet[i].ToString() + " ");
            }
            printDebug("\n");
            SetupReceiveCallback();
        }

        public byte[] GetReceivedData(IAsyncResult ar)
        {
            int nBytesRec = 0;
            try
            {
                //nBytesRec = m_sock.EndReceive(ar);
                nBytesRec = client.EndReceive(ar);
            }
            catch { }
            byte[] byReturn = new byte[nBytesRec];
            Array.Copy(sockSrvRecvBuff, byReturn, nBytesRec);

            /*
            // Check for any remaining data and display it
            // This will improve performance for large packets 
            // but adds nothing to readability and is not essential
            int nToBeRead = m_sock.Available;
            if( nToBeRead > 0 )
            {
                byte [] byData = new byte[nToBeRead];
                m_sock.Receive( byData );
                // Append byData to byReturn here
            }
            */
            return byReturn;
        }
        public delegate void printDebugCallback(string str);
        public void printDebug(string str)
        {
            textBox0.Dispatcher.Invoke(new printDebugCallback(DebugAppend), new object[] { str });
        }

        private void DebugAppend(string str)
        {

            textBox0.AppendText(str);
        }

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            // Convert to byte array and send.
            Byte[] byteDateLine = { Convert.ToByte(sockSrvNumSend.Text) };
            client.Send(byteDateLine, byteDateLine.Length, 0);
        }

        private void Baud_Rates_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }
    }
}

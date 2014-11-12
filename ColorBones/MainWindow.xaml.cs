    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Linq;
    using System.Runtime.InteropServices;
    using System.Text;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Data;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Navigation;
    using System.Windows.Shapes;
    using Microsoft.Kinect;

namespace ColorBones
{
    /// <summary>
    /// Lógica de interacción para MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        /// <summary>
        /// Kinect DPI.
        /// </summary>
        public static readonly double DPI = 96.0;

        /// <summary>
        /// Intermediate storage for receiving frame data from the sensor
        /// </summary>
        private PixelFormat format = PixelFormats.Bgr32;
        private byte[] pixels = null;

        /// <summary>
        /// The bitmap source.
        /// </summary>
        static WriteableBitmap _bitmap = null;

        /// <summary>
        /// Font size of face property text 
        /// </summary>
        private const double DrawTextFontSize = 30;

        /// <summary>
        /// Number of bodies tracked
        /// </summary>
        private int bodyCount;

        /// <summary>
        /// Face frame sources
        /// </summary>
        //private BodyFrameSource[] frameSources = null;

          /// <summary>
        /// Reader for color frames
        /// </summary>
        private MultiSourceFrameReader reader;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private IList<Body> bodies = null;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;


        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
                /// initialize the components (controls) of the window
                this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;


        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // get the kinectSensor object
            kinectSensor = KinectSensor.GetDefault();
            if (kinectSensor != null)
            {
                // open the sensor
                kinectSensor.Open();
                // open the reader for the color frames
                
                reader = kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);
     
                // set IsAvailableChanged event notifier
                kinectSensor.IsAvailableChanged += Sensor_IsAvailableChanged;

                // set the status text
                this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.NoSensorStatusText;
            }
            if (reader != null)
            {
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (reader != null)
            {
                reader.Dispose();
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
            }
        }

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Color
            try
            {
                var frameReference = e.FrameReference.AcquireFrame();
                
                using (var colorFrame = frameReference.ColorFrameReference.AcquireFrame()) {
                    if (colorFrame != null)
                    {
                    // ColorFrame is IDisposable
                        if (_bitmap == null)
                        {
                            // get size of joint space
                            displayWidth = colorFrame.FrameDescription.Width;
                            displayHeight = colorFrame.FrameDescription.Height;
                            pixels = new byte[this.displayWidth * this.displayHeight * ((format.BitsPerPixel + 7) / 8)];
                            _bitmap = new WriteableBitmap(displayWidth, displayHeight, DPI, DPI, format, null);
                        }
                        
                        // verify data and write the new color frame data to the display bitmap
                        if (colorFrame.RawColorImageFormat == ColorImageFormat.Bgra)
                            {
                                colorFrame.CopyRawFrameDataToArray(this.pixels);
                            }
                            else
                            {
                                colorFrame.CopyConvertedFrameDataToArray(this.pixels, ColorImageFormat.Bgra);
                            }
                        _bitmap.Lock();

                        Marshal.Copy(pixels, 0, _bitmap.BackBuffer, pixels.Length);
                        _bitmap.AddDirtyRect(new Int32Rect(0, 0, displayWidth, displayHeight));

                        _bitmap.Unlock();

                            image.Source = _bitmap;
                    }
                }

            // Body
               using (var frame = frameReference.BodyFrameReference.AcquireFrame())
                {
                    if (frame != null)
                    {
                        // BodyFrame is IDisposable
                        canvas.Children.Clear();

                        bodies = new Body[frame.BodyFrameSource.BodyCount];

                        // set the maximum number of bodies that would be tracked by Kinect
                        this.bodyCount = kinectSensor.BodyFrameSource.BodyCount;

                        // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                        // As long as those body objects are not disposed and not set to null in the array,
                        // those body objects will be re-used.
                        frame.GetAndRefreshBodyData(bodies);

                        foreach (Body body in bodies)
                        {
                            if (body.IsTracked)
                            {
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                // convert the joint points to depth (display) space
                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                                foreach (Joint joint in body.Joints.Values)
                                {
                                    if (joint.TrackingState == TrackingState.Tracked)
                                    {
                                        // 3D space point
                                        CameraSpacePoint position = joint.Position;

                                        // 2D space point
                                        Point point = new Point();


                                        ColorSpacePoint colorPoint = kinectSensor.CoordinateMapper.MapCameraPointToColorSpace(position);

                                        point.X = float.IsInfinity(colorPoint.X) ? 0 : colorPoint.X;
                                        point.Y = float.IsInfinity(colorPoint.Y) ? 0 : colorPoint.Y;

                                        jointPoints[joint.JointType] = point;

                                        DrawJoint(canvas, point);
                                    }
                                }
                                //this.DrawRotationResults(canvas, joints[JointType.WristRight], jointPoints[JointType.WristRight].X, jointPoints[JointType.WristRight].Y, body);
                                this.DrawBody(joints, jointPoints, canvas);
                                //this.WriteAngle(AngleBetweenJoints(joints[JointType.WristRight], joints[JointType.ElbowRight], joints[JointType.ShoulderRight]), jointPoints[JointType.ElbowRight].X, jointPoints[JointType.ElbowRight].Y, canvas);

                            }
                        }
                    }
                }
            }
            catch (Exception)
            {
                Console.WriteLine("No he capturado el frame");
            }
        }

        
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, Canvas c)
        {
            // Draw the bones
            // Torso
            this.DrawBone(joints, jointPoints, JointType.Head, JointType.Neck, c);
            this.DrawBone(joints, jointPoints, JointType.Neck, JointType.SpineShoulder, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.SpineMid, c);
            this.DrawBone(joints, jointPoints, JointType.SpineMid, JointType.SpineBase, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderRight, c);
            this.DrawBone(joints, jointPoints, JointType.SpineShoulder, JointType.ShoulderLeft, c);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipRight, c);
            this.DrawBone(joints, jointPoints, JointType.SpineBase, JointType.HipLeft, c);

            // Right Arm    
            this.DrawBone(joints, jointPoints, JointType.ShoulderRight, JointType.ElbowRight, c);
            this.DrawBone(joints, jointPoints, JointType.ElbowRight, JointType.WristRight, c);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.HandRight, c);
            this.DrawBone(joints, jointPoints, JointType.HandRight, JointType.HandTipRight, c);
            this.DrawBone(joints, jointPoints, JointType.WristRight, JointType.ThumbRight, c);

            // Left Arm
            this.DrawBone(joints, jointPoints, JointType.ShoulderLeft, JointType.ElbowLeft, c);
            this.DrawBone(joints, jointPoints, JointType.ElbowLeft, JointType.WristLeft, c);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.HandLeft, c);
            this.DrawBone(joints, jointPoints, JointType.HandLeft, JointType.HandTipLeft, c);
            this.DrawBone(joints, jointPoints, JointType.WristLeft, JointType.ThumbLeft, c);

            // Right Leg
            this.DrawBone(joints, jointPoints, JointType.HipRight, JointType.KneeRight, c);
            this.DrawBone(joints, jointPoints, JointType.KneeRight, JointType.AnkleRight, c);
            this.DrawBone(joints, jointPoints, JointType.AnkleRight, JointType.FootRight, c);

            // Left Leg
            this.DrawBone(joints, jointPoints, JointType.HipLeft, JointType.KneeLeft, c);
            this.DrawBone(joints, jointPoints, JointType.KneeLeft, JointType.AnkleLeft, c);
            this.DrawBone(joints, jointPoints, JointType.AnkleLeft, JointType.FootLeft, c);
        }

        public void DrawJoint(Canvas canvas, Point point)
        {
            // Create an ellipse.
            Ellipse ellipse = new Ellipse
            {
                Width = 20,
                Height = 20,
                Fill = new SolidColorBrush(Colors.LightBlue)
            };

            // Position the ellipse according to the point's coordinates.
            Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, point.Y - ellipse.Height / 2);

            // Add the ellipse to the canvas.
            canvas.Children.Add(ellipse);
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, Canvas c)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];
            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                Line line = new Line
                {
                    X1 = (float)jointPoints[jointType0].X,
                    Y1 = (float)jointPoints[jointType0].Y,
                    X2 = (float)jointPoints[jointType1].X,
                    Y2 = (float)jointPoints[jointType1].Y,
                    StrokeThickness = 8,
                    Stroke = new SolidColorBrush(Colors.LightBlue)
                };

                canvas.Children.Add(line);
            }
        }

        /// <summary>
        /// Regresa el ángulo interno dadas 3 Joints
        /// </summary>
        /// <param name="j1"></param>
        /// <param name="j2"></param>
        /// <param name="j3"></param>
        /// <returns></returns>
        public static double AngleBetweenJoints(Joint j1, Joint j2, Joint j3)
        {
            double Angulo = 0;
            double shrhX = j1.Position.X - j2.Position.X;
            double shrhY = j1.Position.Y - j2.Position.Y;
            double shrhZ = j1.Position.Z - j2.Position.Z;
            double hsl = vectorNorm(shrhX, shrhY, shrhZ);
            double unrhX = j3.Position.X - j2.Position.X;
            double unrhY = j3.Position.Y - j2.Position.Y;
            double unrhZ = j3.Position.Z - j2.Position.Z;
            double hul = vectorNorm(unrhX, unrhY, unrhZ);
            double mhshu = shrhX * unrhX + shrhY * unrhY + shrhZ * unrhZ;
            double x = mhshu / (hul * hsl);
            if (x != Double.NaN)
            {
                if (-1 <= x && x <= 1)
                {
                    double angleRad = Math.Acos(x);
                    Angulo = angleRad * (180.0 / Math.PI);
                }
                else
                    Angulo = 0;
            }
            else
                Angulo = 0;
            return Angulo;
        }

        /// <summary>
        /// Euclidean norm of 3-component Vector
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        private static double vectorNorm(double x, double y, double z)
        {
            return Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2));
        }

        private void WriteAngle(double p, double x, double y, Canvas c)
        {
            TextBlock textBlock = new TextBlock();
            textBlock.Text = string.Format(CultureInfo.CurrentCulture, Properties.Resources.AngleMeasureMessage, p.ToString());
            textBlock.FontSize = 20;
            textBlock.Foreground = new SolidColorBrush(Colors.LightBlue);
            Canvas.SetLeft(textBlock, x);
            Canvas.SetTop(textBlock, y);
            c.Children.Add(textBlock);
        }

        /// <summary>
        /// Draws face frame results
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceResult">container of all face frame results</param>
        /// <param name="drawingContext">drawing context to render to</param>
       private void DrawRotationResults(Canvas can, Joint joint, double x, double y, Body bod)
        {
            string jointText = string.Empty;
           Vector4 quaternion = new Vector4();
            IReadOnlyDictionary<JointType, JointOrientation> orientations = bod.JointOrientations;
               quaternion.X = bod.JointOrientations[joint.JointType].Orientation.X;
               quaternion.Y = bod.JointOrientations[joint.JointType].Orientation.Y;
               quaternion.Z = bod.JointOrientations[joint.JointType].Orientation.Z;
               quaternion.W = bod.JointOrientations[joint.JointType].Orientation.W;
            
                int pitch, yaw, roll;

                ExtractRotationInDegrees(quaternion, out pitch, out yaw, out roll);

                TextBlock textBlock = new TextBlock();
                textBlock.Text += "JointYaw : " + yaw + "\n" +
                            "JointPitch : " + pitch + "\n" +
                            "JointRoll : " + roll + "\n";
                textBlock.FontSize = 20;
                textBlock.Foreground = new SolidColorBrush(Colors.Red);
                Canvas.SetLeft(textBlock, x);
                Canvas.SetTop(textBlock, y);
                can.Children.Add(textBlock);
        }

       /// <summary>
       /// Converts rotation quaternion to Euler angles 
       /// And then maps them to a specified range of values to control the refresh rate
       /// </summary>
       /// <param name="rotQuaternion">face rotation quaternion</param>
       /// <param name="pitch">rotation about the X-axis</param>
       /// <param name="yaw">rotation about the Y-axis</param>
       /// <param name="roll">rotation about the Z-axis</param>
       private static void ExtractRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll)
       {
           double x = rotQuaternion.X;
           double y = rotQuaternion.Y;
           double z = rotQuaternion.Z;
           double w = rotQuaternion.W;

           // convierte la rotacion a angulos de Euler, en grados
           double yawD, pitchD, rollD;
           pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
           yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
           rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

           // limita los valores a un múltiplo de un incremento específico que controle el ratio de actualizaciones
           double increment = 1.0;
           pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
           yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
           roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
       }

    }

}

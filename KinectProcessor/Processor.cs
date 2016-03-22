using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using SJTU.IOTLab.RoomBuilder.Struct;
using KdTree;
using KdTree.Math;

namespace SJTU.IOTLab.RoomBuilder.KinectProcessor
{
    public delegate void SetStatusText(string text);

    class Processor
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        private const int MapPixelWidth = 1000;
        private const int MapPixelHeight = 1000;
        private const double MapActualWidth = 6f;   // 6m
        private const double MapActualHeight = 6f;  // 6m
        private const int MAX_FPS = 15;

        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        public WriteableBitmap depthBitmap = null;
        public WriteableBitmap splittedFlatBitmap = null;

        private PointProcessor pointProcessor = null;
        private KdTree<double, int> pointCloud = null;
        private bool cloudInitialized = false;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;
        private byte[] floatPixels = null;

        private SetStatusText setStatusText = null;
        private DateTime timestamp;

        public static void startPlayback()
        {
            PlaybackHelper.PlaybackClip("D:\\Dev\\warehouse_1.xef", 10);
        }

        public void initialize(SetStatusText setStatusText)
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
            this.floatPixels = new byte[MapPixelWidth * MapPixelHeight * 3];

            this.pointCloud = new KdTree<double, int>(2, new DoubleMath());

            // create the bitmap to display
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
            this.splittedFlatBitmap = new WriteableBitmap(MapPixelWidth, MapPixelHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            this.pointProcessor = new PointProcessor((ushort)this.depthFrameDescription.Width, (ushort)this.depthFrameDescription.Height);
            this.pointProcessor.setCameraLocation(new rPoint(0, -3, 0), Math.PI / 2f);

            this.setStatusText = setStatusText;

            this.timestamp = DateTime.Now;

            // set the status text
            this.setStatusText(this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                             : Properties.Resources.NoSensorStatusText);

            if (!this.kinectSensor.IsAvailable)
            {
                Thread playbackThread = new Thread(startPlayback);
                playbackThread.Start();
            }
        }

        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            TimeSpan elapsedSpan = new TimeSpan(DateTime.Now.Ticks - this.timestamp.Ticks);
            if (elapsedSpan.Milliseconds < (1000f / MAX_FPS)) return;
            this.timestamp = DateTime.Now;

            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;
                            //ushort maxDepth = 2500;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
                this.RenderFlatBitmap();
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // TODO: clear point cloud for now.
            this.splittedFlatBitmap.Lock();
            this.splittedFlatBitmap.Clear();
            uint* flatBitmapPixelsPointer = (uint*)this.splittedFlatBitmap.BackBuffer;

            // convert depth to a visual representation
            // and transform pixels to point cloud
            for (ushort y = 0; y < depthFrameDescription.Height; ++y)
            {
                for (ushort x = 0; x < depthFrameDescription.Width; ++x)
                {
                    int index = x + y * depthFrameDescription.Width;
                    // Get the depth for this pixel
                    ushort depth = frameData[index];

                    // To convert to a byte, we're mapping the depth value to the byte range.
                    // Values outside the reliable depth range are mapped to 0 (black).
                    this.depthPixels[index] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);

                    if (depth >= minDepth && depth <= maxDepth)
                    {
                        Pixel pixel = new Pixel(x, y, depth);
                        double z = pointProcessor.getZ(pixel);
                        uint color;

                        //if (z > -.55f && z < .85f) color = 0xff0000ff;
                        if (z > -.65f && z < -.35f) color = 0xffff0000;
                        else if (z > -.05f && z < .25f) color = 0xff00ff00;
                        else if (z > .55f && z < .85f) color = 0xff0000ff;
                        else color = 0;

                        if (color != 0)
                        {
                            rPoint point = (rPoint)(pointProcessor.transform(pixel));
                            if (!cloudInitialized) pointCloud.Add(new double[2] { point.x, point.y }, 0);
                            int mapX = (int)((MapActualWidth / 2f + point.x) / MapActualWidth * MapPixelWidth);
                            int mapY = (int)((MapActualHeight / 2f - point.y) / MapActualHeight * MapPixelHeight);

                            if (mapX >= 0 && mapX < MapPixelWidth && mapY >= 0 && mapY < MapPixelHeight)
                            {
                                // Treat the color data as 4-byte pixels
                                flatBitmapPixelsPointer[mapY * MapPixelWidth + mapX] = color;
                            }
                        }
                    }
                }
            }

            cloudInitialized = true;
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }

        private void RenderFlatBitmap()
        {
            //renderKdTree2D(pointCloud.root);
            this.splittedFlatBitmap.AddDirtyRect(new Int32Rect(0, 0, this.splittedFlatBitmap.PixelWidth, this.splittedFlatBitmap.PixelHeight));
            this.splittedFlatBitmap.Unlock();
        }

        private unsafe void renderKdTree2D(KdTreeNode<double, int> node)
        {
            if (node == null) return;
            uint* flatBitmapPixelsPointer = (uint*)this.splittedFlatBitmap.BackBuffer;
            rPoint2d point = new rPoint2d(node.Point[0], node.Point[1]);

            int mapX = (int)((MapActualWidth / 2f + point.x) / MapActualWidth * MapPixelWidth);
            int mapY = (int)((MapActualHeight / 2f - point.y) / MapActualHeight * MapPixelHeight);

            if (mapX >= 0 && mapX < MapPixelWidth && mapY >= 0 && mapY < MapPixelHeight)
            {
                // Treat the color data as 4-byte pixels
                flatBitmapPixelsPointer[mapY * MapPixelWidth + mapX] = 0xffff0000;
            }

            //renderKdTree2D(node.leftChild);
            //renderKdTree2D(node.rightChild);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.setStatusText(this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                             : Properties.Resources.SensorNotAvailableStatusText);
        }

        public void tryClosing()
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
    }
}

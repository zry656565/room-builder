using System;

namespace SJTU.IOTLab.RoomBuilder.Struct
{
    class PointProcessor
    {
        /***
         * f = w / (2tan(fov/2))
         * Referrence
         * - http://smeenk.com/kinect-field-of-view-comparison/
         * - http://stackoverflow.com/questions/17832238/kinect-intrinsic-parameters-from-field-of-view
         */
        private const float FOCAL_LENGTH_IN_PIXELS_X = 361.6f; // 512 / (2 * tan(70.6/2))
        private const float FOCAL_LENGTH_IN_PIXELS_Y = 367.2f; // 424 / (2 * tan(60/2))
        private const float FOCAL_LENGTH_IN_PIXELS = 364.4f;  // f = sqrt((fx^2 + fy^2)/2)

        private int depthImageWidth;
        private int depthImageHeight;

        private rPoint cameraLocation;
        private double cameraOrientation;

        public PointProcessor(int width, int height)
        {
            this.depthImageHeight = height;
            this.depthImageWidth = width;
            this.cameraLocation = new rPoint(0, 0, 0);
            this.cameraOrientation = Math.PI / 2;
        }

        public void setCameraLocation(rPoint location, double orientation)
        {
            this.cameraLocation = location;
            this.cameraOrientation = orientation;
        }

        public rPoint? transform(Pixel pixel, double zStart = -1, double zEnd = -1)
        {
            double screenX = pixel.x - this.depthImageWidth / 2f;
            double screenY = pixel.y - this.depthImageHeight / 2f;
            double depth = pixel.depth / 1000f;
            double rate = depth / FOCAL_LENGTH_IN_PIXELS;

            double z = this.cameraLocation.z - screenY / FOCAL_LENGTH_IN_PIXELS * depth;

            if (zStart == -1 || (z >= zStart && z < zEnd))
            {
                // 小孔成像，X轴反转
                return new rPoint(
                    this.cameraLocation.x - (depth * Math.Cos(this.cameraOrientation) + screenX * Math.Sin(this.cameraOrientation) * rate),
                    this.cameraLocation.y + depth * Math.Sin(this.cameraOrientation) - screenX * Math.Cos(this.cameraOrientation) * rate,
                    z);
            }
            else return null;
        }

        public rPoint? transform(int x, int y, int depth, double zStart = -1, double zEnd = -1)
        {
            return transform(new Pixel(x, y, depth), zStart, zEnd);
        }

        public double getZ(Pixel pixel)
        {
            double screenY = pixel.y - this.depthImageHeight / 2f;
            double depth = pixel.depth / 1000f;
            return this.cameraLocation.z - screenY / FOCAL_LENGTH_IN_PIXELS * depth;
        }

        public double getZ(int x, int y, int depth)
        {
            return getZ(new Pixel(x, y, depth));
        }
    }

    public struct Pixel
    {
        public int x;
        public int y;
        public int depth;

        public Pixel(int x, int y, int depth)
        {
            this.x = x;
            this.y = y;
            this.depth = depth;
        }
    }

    public struct rPoint
    {
        public double x;
        public double y;
        public double z;

        public rPoint(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    public struct rPoint2d
    {
        public double x;
        public double y;

        public rPoint2d(double x, double y)
        {
            this.x = x;
            this.y = y;
        }
    }
}
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using KdTree;

namespace SJTU.IOTLab.RoomBuilder.Struct
{
    // simple implementation of 2-d tree
    class K2dTree
    {
        private K2dTreeNode root;
        enum AXIS { X, Y };

        public K2dTree(r2Point[] points)
        {
            root = createTree(points, 0);
        }

        private K2dTreeNode createTree(r2Point[] points, int callDepth)
        {
            if (points.Length == 0) return null;

            AXIS axis = (AXIS)(callDepth % 2);
            int mid = (int)Math.Floor(points.Length / 2.0f);

            K2dTreeNode node = new K2dTreeNode();
            node.location = points[mid];

            List<r2Point> leftPoints = new List<r2Point>();
            List<r2Point> rightPoints = new List<r2Point>();

            for (int i = 0; i < points.Length; i++)
            {
                if (i == mid) continue;
                if (axis == AXIS.X)
                {
                    (points[i].x < points[mid].x ? leftPoints : rightPoints).Add(points[i]);
                }
                else
                {
                    (points[i].y < points[mid].y ? leftPoints : rightPoints).Add(points[i]);
                }
            }

            node.left = createTree(leftPoints.ToArray(), callDepth + 1);
            node.right = createTree(rightPoints.ToArray(), callDepth + 1);
            return node;
        }
    }

    class K2dTreeNode
    {
        public r2Point location;
        public K2dTreeNode left;
        public K2dTreeNode right;
    }
}

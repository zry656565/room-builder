using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SJTU.IOTLab.RoomBuilder.Struct
{
    // simple implementation of 3-d tree
    class K3dTree
    {
        private KdTreeNode root;
        enum AXIS { X, Y, Z };

        public K3dTree(rPoint[] points)
        {
            root = createTree(points, 0);
        }

        public KdTreeNode createTree(rPoint[] points, int callDepth) {
            AXIS axis = (AXIS)(callDepth % 3);
            int index = (int)Math.Floor(points.Length / 2.0f);

            KdTreeNode node = new KdTreeNode();
            node.location = points[index];
            node.left = createTree(points.Take(index).ToArray(), callDepth + 1);
            node.right = createTree(points.Skip(index + 1).ToArray(), callDepth + 1);
            return node;
        }
    }

    class KdTreeNode
    {
        public rPoint location;
        public KdTreeNode left;
        public KdTreeNode right;
    }
}

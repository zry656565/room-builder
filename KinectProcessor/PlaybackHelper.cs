using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Kinect.Tools;

namespace SJTU.IOTLab.RoomBuilder.KinectProcessor
{
    class PlaybackHelper
    {
        public static void PlaybackClip(string filePath, uint loopCount)
        {
            using (KStudioClient client = KStudio.CreateClient())
            {
               client.ConnectToService();
               
               using (KStudioPlayback playback = client.CreatePlayback(filePath))
               {
                   playback.LoopCount = loopCount;
                   playback.Start();
        
                   while (playback.State == KStudioPlaybackState.Playing)
                   {
                       Thread.Sleep(500);
                   }
        
                   if (playback.State == KStudioPlaybackState.Error)
                   {
                       throw new InvalidOperationException("Error: Playback failed!");
                   }
               }
        
               client.DisconnectFromService();
            }
        }
    }
}

#!/usr/bin/env python   
import rospy
import time
import os
import stat
import signal,psutil
import sys
from datetime import datetime
from rosbag import rosbag_main
from multiprocessing import Process,RLock
from k2_client.msg import Body,BodyArray

class SkeletonRosbag(object):


    def __init__(self):
        self.sub = rospy.Subscriber('/head/kinect2/k2_bodies/bodies',BodyArray,self.process_message)
        self.recording_status=False
        self.writing_process=None
        self.last_message_time=[]
        self.timer=None
        self.lock=RLock()

    def start_recording(self):

        n=datetime.now()
        nowstring =\
        str(n.year).zfill(4)+str(n.month).zfill(2)+str(n.day).zfill(2)+str(n.hour).zfill(2)+str(n.minute).zfill(2)+str(n.second).zfill(2)
        rosbag_main.record_cmd
        self.writing_process = Process(target=rosbag_main.record_cmd,args = \
                                       [['/head/kinect2/k2_rgb/image/compressed',\
                                         '/head/kinect2/k2_depth/image',\
                                         '/head/kinect2/k2_depth/camera_info',\
                                         '/head/kinect2/k2_bodies/bodies',\
                                         '/head/kinect2/k2_audio/audio',\
                                         '-Orecording_'+nowstring]])
        self.writing_process.start()
        print "recording started at time: "+nowstring

    
    def stop_recording(self,event):
        with self.lock:
            try:

                parent=psutil.Process(self.writing_process.pid)
                children= parent.get_children(recursive=True)
                for child_process in children:
                    child_process.send_signal(signal.SIGINT)
            except psutil.NoSuchProcess:
                pass
            #  os.kill(int(self.writing_process.pid),signal.SIGINT)
            self.timer=None
        print "recording stopped"


    def process_message(self,data):
        #  print "body message arrived"
        with self.lock:
            #  print self.timer
            if self.timer == None:
                self.start_recording()
            else:
                self.timer.shutdown()
            self.timer=rospy.Timer(rospy.Duration(10),self.stop_recording,True)






def main():
    writer=SkeletonRosbag()
    rospy.init_node('skeleton_recorder', anonymous=True)


    try:
        rospy.spin()
              
    except KeyboardInterrupt:
       print 'Shutting down'   






if __name__ == '__main__':
    main()


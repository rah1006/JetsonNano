#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import numpy as np
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D

"""TODO

I pretended the stop sign (which the detectnet model is trained to detect) was a "landing pad".
Tell Thein she can print out and mount stop signs to her other vehicles and this camera can track them and talk to ROS.

def whereAmI():
    some function that knows where the camera is. The D435i camera has am IMU but I couldn't get it working on the Jetson Nano.
    Probably need to subscribe to the rostopic camera/imu or something.

def whereIsTarget():
    Find out where the target is. The depth camera does output to a ROS topic. I'll explore parsing and processing that if I have time.
    The code below tells you the X and Y pixels of the stop sign.
    If camera is mounted on quad pointing down, like I anticipate, you may not have to consider trig to find XYZ waypoint of target.

def tellWaypoint():
    publish a new topic with only XYZ waypoint coordinates of the object. This will likely be w.r.t. the camera so you may need to convert it to the world frame (from the camera frame). You may even want to consider the difference between camera IMU and quad IMU.


-Ricky Heath 4/29/2022
rickyh2007@gmail.com
yes you can contact me with questions.

"""

#Testing getting depth data from the D435i camera
def convert_depth_image(ros_image,xpixeltarget,ypixeltarget):
#https://www.neowin.net/forum/topic/1237820-how-to-calculate-the-ppi-from-resolution-x-and-y/
    print("convert_depth_image running")
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
      depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
      depth_array = np.array(depth_image, dtype=np.float32) #720 x 1280 array... are x and y swapped?
      #print(np.transpose(depth_array).shape) #I think they mixed up X and Y resolution lol
      depth_array = np.transpose(depth_array) #units are supposedly mm
      #This is where we want the depth of the center of our target. We could also consider, for uneven-surface targets, parsing the whole bounding box for the "shortest" depth

      ztarget = depth_array[round(xpixeltarget),round(ypixeltarget)]
      print("publishing to targetXpYpZ")
      pub = rospy.Publisher('/targetXpYpZ', String, queue_size=10)
      outdata2 = [xpixeltarget,ypixeltarget,ztarget]
      pub.publish(str(outdata2))

      #center_idx = np.array(depth_array.shape) / 2
      #print ('center depth:', depth_array[center_idx[0], center_idx[1]])#Convert the depth image using the default passthrough encoding

    except CvBridgeError, e:
       print e
     #Convert the depth image to a Numpy array
    depth_array = np.array(depth_image, dtype=np.float32)

    #rospy.loginfo(depth_array)
    #return depth_array

def pixel2depth(xpixeltarget,ypixeltarget):
	#rospy.init_node('pixel2depth',anonymous=True)
        print('pixel2depth running')
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()

def callback1(msg):
    #This is the part where we parse through and process the detectnet output data

    #This still needs a fix - if multiple objects are detected it only looks at the first one [0]
    #I believe if we do a for loop for the length of msg.detections, we can

    #mydata = msg.detections[0].results[0]   
    #rospy.loginfo(msg.detections[0].results[0])
    #print('msg.det type = ', type(msg.detections) ) #this is a list
    print('msg.det length = ',len(msg.detections) ) #this tells you how many objects detected

    confidenceThreshold = 0.75 #I want this model to be 75+% sure it is a stop sign

    #for i in range(1): #this needs to be the number of objects detected
    
    for i in range(len(msg.detections)):
      mydata=msg.detections[i]
      myid = mydata.results[0].id #stop sign id is 13 in the label list
      myscore=mydata.results[0].score
      #mostly interested in id (detection) and score (confidence that the object is correctly labeled)
      myboxox = mydata.bbox.center.x
      myboxoy = mydata.bbox.center.y
      print('id, confidence, x, y =',myid, myscore, myboxox, myboxoy)
      outdata = [myid, myscore, myboxox, myboxoy]
      
      if (myid == 13 and myscore > confidenceThreshold): #might need to be '13'
        pub = rospy.Publisher('/targetData', String, queue_size=10)
        pub.publish(str(outdata))
        #pixel2depth(outdata[2],outdata[3]) #I couldn't get depth data publishing while detectnet was open
  
   #return outdata
    #rospy.loginfo(mydata)
    #rospy.sleep(1) #pause
    #I reverse-engineered the data indexing through trial-and-error and a lot of luck

def messenger():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('talkerRH', anonymous=True)
    rospy.Subscriber('detectnet/detections', Detection2DArray, callback1)
	#will likely want this to be listening to detectnet/detection

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    #thankful for online forums
    #while not rospy.is_shutdown():
    # do whatever you want here
        #rate = rospy.Rate(1) # 10hz
        #pub.publish(str(outdata))
        #rospy.sleep(1)


if __name__ == '__main__':
    try:
        messenger()        
    except rospy.ROSInterruptException:
        pass

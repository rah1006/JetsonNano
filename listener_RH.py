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

#I did not end up using this in my bash script, since the talker_RH.py script both subscribes AND publishes!
#Instead I tried to hijack this tutorial-based script to save a ROStopic to a local file.

import numpy as np
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError

#You may have to import more types as you encounter them!g
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import Image

#note that raw_input gets a string in Python 2, which I am accidentally running instead of Python 3 lol
print("Welcome! If this fails, go to the script and ensure the rostopic type is imported")
myTopic = raw_input("Run 'rostopic list' and type which topic you want to save:")
topicType = input("Run 'rostopic info "+myTopic+"' and enter the topic type:") #Detection2DArray is detectnet type
Ts = 1
print("kk just press 'ctrl+c' to kill this program when you've gotten your data.")

def callback(msg):
    """OUT WITH THE OLD
    #mydata = msg.detections[0].results[0]   
    #rospy.loginfo(msg.detections[0].results[0])
    mydata=msg.detections[0]
    myid = mydata.results[0].id #stop sign id is 13 in the label list
    myscore=mydata.results[0].score
    #mostly interested in id (detection) and score (confidence that the object is correctly labeled)
    myboxox = mydata.bbox.center.x
    myboxoy = mydata.bbox.center.y
    print('id, confidence, x, y =',myid, myscore, myboxox, myboxoy)
    #rospy.loginfo(mydata)
    rospy.sleep(1) #pause
    #I reverse-engineered the data indexing through trial-and-error and a lot of luck"""

    #IN WITH THE NEW - I want to save the raw output to a .txt file
    #a_list = ["abc", "def", "ghi"]
    textfile = open("output.txt", "a") #a = append, w = overwrite
    textfile.write("We just got a letter! I wonder who it's from?\n")
    textfile.write(str(msg))
    textfile.write("\n")
    textfile.close()
    time.sleep(Ts)
    #for element in msg:
    #  textfile.write(element + "\n")
    #  textfile.close()
    #https://www.adamsmith.haus/python/answers/how-to-write-a-list-to-a-file-in-python

def callback2(msg):
    #create your own custom callback here then replace the Subscriber line in listener()
    print("length",len(msg.data))
    #print("shape",msg.data.shape)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listenerRH', anonymous=True)
    #rospy.Subscriber('detectnet/detections', Detection2DArray, callback)
	#will likely want this to be listening to detectnet/detections
    rospy.Subscriber(myTopic, topicType, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#Testing getting depth data from the D435i camera
def convert_depth_image(ros_image):
    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
      depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
      depth_array = np.array(depth_image, dtype=np.float32) #720 x 1280 array... are x and y swapped?
      #print(np.transpose(depth_array).shape) #I think they mixed up X and Y resolution lol
      center_idx = np.array(depth_array.shape) / 2
      print ('center depth:', depth_array[center_idx[0], center_idx[1]])#Convert the depth image using the default passthrough encoding

    except CvBridgeError, e:
       print e
     #Convert the depth image to a Numpy array
    depth_array = np.array(depth_image, dtype=np.float32)

    #rospy.loginfo(depth_array)
    #return depth_array

def pixel2depth():
	rospy.init_node('pixel2depth',anonymous=True)
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
	rospy.spin()

if __name__ == '__main__':
    listener()
    #pixel2depth()




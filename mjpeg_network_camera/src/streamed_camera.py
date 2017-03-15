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

import rospy
from sensor_msgs.msg import CompressedImage
import httplib
import sys

# Initalize ROS
rospy.init_node('streamed_network_camera')
pub = rospy.Publisher('/camera_kodak_sp360/image/compressed', CompressedImage, queue_size=0)

# Get parameters
host = rospy.get_param('~host', '172.16.0.254')
port = int(rospy.get_param('~port', '9176'))
request = rospy.get_param('~request', '')
frame_id = rospy.get_param('~frame_id', 'camera_kodak_sp360')

# Connect to camera
h = httplib.HTTP(host, port)
h.putrequest('GET', request)
h.endheaders()

(code, msg, header) = h.getreply()
if(code != 200): # 200 is the http response for OK
	rospy.logerr("Error connecting to stream.")
	rospy.loginfo("Tried to connect to: %s:%s with GET %s", host, port, request)
        rospy.loginfo("\n\t%s \n\n%s", msg, header) 
        sys.exit(1)

# Open socket like a file
f = h.getfile()

while not rospy.is_shutdown():
        # Find start of image
        while not rospy.is_shutdown():
                if(f.readline() == 'Content-type: image/jpeg\n'):
                        break
                        
        # Next line is Content-Length:
        lenline = f.readline()
        length = int(filter(lambda x: x.isdigit(), lenline))

	# Next line is empty, then the image starts
	f.readline()
        
	msg = CompressedImage()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = frame_id
	msg.format = "jpeg"
	msg.data = f.read(length) # Read the image data
	pub.publish(msg)

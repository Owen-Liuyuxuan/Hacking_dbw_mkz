#! /usr/bin/env python

import cv2
import numpy as np
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import CameraInfo
import rospy
import tf
from cv_bridge import CvBridge, CvBridgeError

number_of_received = 0
number_to_receive = 10
index = 0

my_bridge = CvBridge()
def image_callback(data):
    """
    data: Image
    """
    #image = np.array(data.data, dtype = np.uint8)
    #print(len(data.data)) 
    #print(data.height)
    #print(data.width)
    #cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #wtf = np.array(data.data, dtype = np.uint8)
    
    global index
    global number_of_received
    if index < 250:
        index += 1
        return
    else:
        index = 0
        height = data.height
        width = data.width
        new_image = my_bridge.imgmsg_to_cv2(data, "bgr8")
        #new_image = np.zeros([width, height, 3],dtype = np.uint8)
        cv2.imwrite("/home/owen/Desktop/play_ground/src/my_launch_car_sim/scripts/pictures/test_image" +
                    str(number_of_received + 5) + ".png", new_image)
        number_of_received += 1
	print "get one picture"
    
    if(number_of_received >= number_to_receive):
        print("finished")
    




def TF_callback(data):
    pass


def camera_info_callback(data):
    pass


rospy.init_node("image_catcher")
rospy.Subscriber("/vehicle/front_camera/image_raw", Image, image_callback)
listener = tf.TransformListener()
try:
    (trans, rot) = listener.lookupTransform(
        'vehicle/front_camera', 'vehicle/base_footprint', rospy.Time(0))
    print("trans:")
    print(trans)
    print("rot:")
    print(rot)
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    pass
    print("didn't get anything")
rospy.spin()

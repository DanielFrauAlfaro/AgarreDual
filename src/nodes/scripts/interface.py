#! /usr/bin/python3

import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Bridge from ROS Image message to OpenCV
bridge = CvBridge()

# Resize dimensions
dim = (400, 240)

# List of images
images = [[None, None, None],
          [None, None, None]]

# List of flags
start = [[False, False, False], 
         [False, False, False]]

# Image callbacks: raise esach flag, transform the message and adds borders
def robot1_y_camera_cb(data):
    global dim, images, bridge, start, space

    start[0][0] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,2.5,cv2.BORDER_CONSTANT,value = [0,0,255])

    images[0][0] = cv2.resize(im, dim)

def robot1_x_camera_cb(data):
    global dim, images, bridge, start

    start[0][1] = True
    im = bridge.imgmsg_to_cv2(data)
    
    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [0,0,255])

    images[0][1] = cv2.resize(im, dim)

def robot1_tool_camera_cb(data):
    global dim, images, bridge, start

    start[0][2] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [0,0,255])

    images[0][2] = cv2.resize(im, dim)

def robot2_y_camera_cb(data):
    global dim, images, bridge, start

    start[1][0] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [255,0,0])

    images[1][0] = cv2.resize(im, dim)

def robot2_x_camera_cb(data):
    global dim, images, bridge, start

    start[1][1] = True
    im = bridge.imgmsg_to_cv2(data)
    
    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [255,0,0])

    images[1][1] = cv2.resize(im, dim)

def robot2_tool_camera_cb(data):
    global dim, images, bridge, start

    start[1][2] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [255,0,0])

    images[1][2] = cv2.resize(im, dim)


# ---- Main ----
if __name__ == "__main__":
    
    # Arguments
    name1 = sys.argv[1]
    name2 = sys.argv[2]
    n = int(sys.argv[3])
    
    # Node
    rospy.init_node("interface")

    # -------- Subscribers ---------
    # Image topics for first robots
    rospy.Subscriber("/" + name1 + "/y_robot_camera/image_raw", Image, robot1_y_camera_cb)
    rospy.Subscriber("/" + name1 + "/x_robot_camera/image_raw", Image, robot1_x_camera_cb)
    rospy.Subscriber("/" + name1 + "/tool_robot_camera/image_raw", Image, robot1_tool_camera_cb)

    # If there are two robots, subscribe to the second robot's image topics
    if n == 2:
        rospy.Subscriber("/" + name2 + "/y_robot_camera/image_raw", Image, robot2_y_camera_cb)
        rospy.Subscriber("/" + name2 + "/x_robot_camera/image_raw", Image, robot2_x_camera_cb)
        rospy.Subscriber("/" + name2 + "/tool_robot_camera/image_raw", Image, robot2_tool_camera_cb)


    # --- Infinite loop ---
    while not rospy.is_shutdown():
        
        # If all flags are raised ...
        if (n == 1 and start[0] == [True, True, True]) or (n == 2 and start == [[True, True, True], [True, True, True]]):
            
            # Concatenates all images
            im_h_resize = cv2.hconcat([images[0][0], images[0][1], images[0][2]])
            
            if n == 2:
                im_h_resize = cv2.vconcat(im_h_resize, cv2.hconcat([images[1][0], images[1][1], images[1][2]]))

            # Shows the information
            cv2.imshow("Test", im_h_resize)
        
        cv2.waitKey(1)
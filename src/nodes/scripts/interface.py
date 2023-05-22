#! /usr/bin/python3

import cv2
import numpy as np
import rospy
import sys
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Bridge from ROS Image message to OpenCV
bridge = CvBridge()

# Resize dimensions
dim = (400, 240)

# List of images
images = [[None, None],
          [None, None]]

# List of flags
start = [[False, False], 
         [False, False]]

show = True

# Image callbacks: raise esach flag, transform the message and adds borders
def robot1_y_camera_cb(data):
    global dim, images, bridge, start, space

    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [0,0,255])

    images[0][0] = cv2.resize(im, dim)

    start[0][0] = True


def robot1_tool_camera_cb(data):
    global dim, images, bridge, start

    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [0,0,255])

    images[0][1] = cv2.resize(im, dim)

    start[0][1] = True

def robot2_y_camera_cb(data):
    global dim, images, bridge, start

    start[1][0] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [255,0,0])

    images[1][0] = cv2.resize(im, dim)

def robot2_tool_camera_cb(data):
    global dim, images, bridge, start

    start[1][1] = True
    im = bridge.imgmsg_to_cv2(data)

    im= cv2.copyMakeBorder(im,5,5,5,5,cv2.BORDER_CONSTANT,value = [255,0,0])

    images[1][1] = cv2.resize(im, dim)


# Callbacks for interface activation
def interface_cb(data):
    global show

    show = data.data



# ---- Main ----
if __name__ == "__main__":
    
    # Arguments
    name1 = sys.argv[1]
    name2 = sys.argv[2]
    n = int(sys.argv[3])
    
    # Node
    rospy.init_node("interface")

    # --------- Subscribers ---------
    # Image topics for first robots
    rospy.Subscriber("/" + name1 + "/y_robot_camera/image_raw", Image, robot1_y_camera_cb)
    rospy.Subscriber("/" + name1 + "/tool_robot_camera/image_raw", Image, robot1_tool_camera_cb)

    # Interface activation callback
    rospy.Subscriber("/interface", Bool, interface_cb)

    # If there are two robots, subscribe to the second robot's image topics
    if n == 2:
        rospy.Subscriber("/" + name2 + "/y_robot_camera/image_raw", Image, robot2_y_camera_cb)
        rospy.Subscriber("/" + name2 + "/tool_robot_camera/image_raw", Image, robot2_tool_camera_cb)


    # --- Infinite loop ---
    while not rospy.is_shutdown():
        
        # If all flags are raised ...
        if (n == 1 and start[0] == [True, True]) or (n == 2 and start == [[True, True], [True, True]]):
            if show:
                # Concatenates all images
                if n == 2:
                    aux1 = cv2.hconcat([images[0][0],  images[0][1]])
                    aux2 = cv2.hconcat([images[1][0],  images[1][1]])

                    im_h_resize = cv2.vconcat([aux1, aux2])

                elif n == 1:
                    im_h_resize = cv2.hconcat([images[0][0],  images[0][1]])

            # Shows the information
                cv2.imshow("Robots Video", im_h_resize)
            
            else:
                cv2.destroyAllWindows()
        
        cv2.waitKey(1)

    cv2.destroyAllWindows()
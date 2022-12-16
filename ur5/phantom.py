#! /usr/bin/python3

from geometry_msgs.msg import PoseStamped, Pose, WrenchStamped
from sensor_msgs.msg import Joy
import rospy

# Button: sensor_msgs/Joy /arm/button1    /arm/button2

# Wrench: geometry_msgs/WrenchStamped   /arm/servo_cf
# Fuerza: y = 0.9
'''
x_limit: 0.09 - -0.07           * 2
y_limit: 0.2                    * 2
z_limit: 0.19  - -0.075         * 0.4 // *7
'''

rospy.init_node("phantom_ctr")
pub = rospy.Publisher("/pose", Pose, queue_size=10)
pub_f = rospy.Publisher("/arm/servo_cf", WrenchStamped)

pose = Pose()

scale_x = 2
scale_y = 2
scale_z1 = 0.4
scale_z2 = 7

prev_x, prev_y, prev_z = 0.5095, 0.1334, 0.7347
prev_roll, prev_pitch, prev_yaw = 1.57225399 , 1.07079575, -0.001661

xyz = True
vel_control = False
change = False

prev_pose_phantom = Pose()
act_pose_phantom = Pose()

prev_pose_phantom.position.x = 0.0
prev_pose_phantom.position.y = 0.0
prev_pose_phantom.position.z = 0.0


def cart_cb(data):
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    
    prev_x = data.position.x
    prev_y = data.position.y
    prev_z = data.position.z
    
    prev_roll = data.orientation.x
    prev_pitch = data.orientation.y
    prev_yaw = data.orientation.z

def cb(data):
    global pub, pose, scale_x, scale_y, scale_z1, scale_z2
    global prev_x, prev_y, prev_z
    global prev_roll, prev_pitch, prev_yaw
    global prev_pose_phantom, act_pose_phantom
    
    act_pose_phantom.position = data.pose.position
    
    if vel_control == False:
        if xyz:
            pose.position.x = data.pose.position.z * scale_x + 0.5095
            pose.position.y = data.pose.position.x * scale_y + 0.1334
            
            if pose.position.z > 0:
                pose.position.z = data.pose.position.y * scale_z1 + 0.7347
            else:
                pose.position.z = data.pose.position.y * scale_z2 + 0.7347
            
            pose.orientation.x = prev_roll
            pose.orientation.y = prev_pitch
            pose.orientation.z = prev_yaw
            
            prev_pose_phantom.position = data.pose.position
            
            
        else:
            pose.position.x = prev_x
            pose.position.y = prev_y
            pose.position.z = prev_z

            pose.orientation.x = data.pose.position.z + 1.57225399
            pose.orientation.y = data.pose.position.x + 1.07079575
            pose.orientation.z = data.pose.position.y + -0.001661
            
            prev_pose_phantom.orientation.x = data.pose.position.x
            prev_pose_phantom.orientation.y = data.pose.position.y
            prev_pose_phantom.orientation.z = data.pose.position.z
                
        
    
    else:
        if xyz:
            pose.position.x = data.pose.position.z
            pose.position.y = data.pose.position.x
            
            if pose.position.z > 0:
                pose.position.z = data.pose.position.y
            else:
                pose.position.z = data.pose.position.y
            
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            
            
        else:
            pose.position.x = 0.0
            pose.position.y = 0.0
            pose.position.z = 0.0

            pose.orientation.x = data.pose.position.z
            pose.orientation.y = data.pose.position.x 
            pose.orientation.z = data.pose.position.y
        
        
    
    

def cb_bt1(data):
    global xyz, change
    
    if data.buttons == 1:
        change = True
        
        if xyz:
            xyz = False
        else:  
            xyz = True

def cb_bt2(data):
    global vel_control, change
    
    if data.buttons == 1:
        change = True
        
        if vel_control:
            vel_control = False
        else:  
            vel_control = True


rospy.Subscriber("/arm/measured_cp", PoseStamped,cb)
rospy.Subscriber("/cart_pos", Pose, cart_cb)
rospy.Subscriber("/arm/button1", Joy, cb_bt1)
rospy.Subscriber("/arm/button2", Joy, cb_bt2)

wrench = WrenchStamped()

r = rospy.Rate(10)
while True:
    if vel_control:
        wrench.wrench.force.x = (0.0 - act_pose_phantom.position.x)
        wrench.wrench.force.y = (0.0 - act_pose_phantom.position.y)
        wrench.wrench.force.z = (0.0 - act_pose_phantom.position.z)
        pub_f.publish(wrench)
    
    if change:
        print("change")
        if vel_control == False:
            if xyz:
                wrench.wrench.force.x = (prev_pose_phantom.position.x - act_pose_phantom.position.x)
                wrench.wrench.force.y = (prev_pose_phantom.position.y - act_pose_phantom.position.y)
                wrench.wrench.force.z = (prev_pose_phantom.position.z - act_pose_phantom.position.z)
                
                if (prev_pose_phantom.position.x - act_pose_phantom.position.x) < 0.001 and (prev_pose_phantom.position.y - act_pose_phantom.position.y) < 0.001 and (prev_pose_phantom.position.z - act_pose_phantom.position.z) < 0.001:
                    change = False
            else:
                wrench.wrench.force.x = (prev_pose_phantom.orientation.x - act_pose_phantom.position.x)
                wrench.wrench.force.y = (prev_pose_phantom.orientation.y - act_pose_phantom.position.y)
                wrench.wrench.force.z = (prev_pose_phantom.orientation.z - act_pose_phantom.position.z)
                
                if (prev_pose_phantom.orientation.x - act_pose_phantom.position.x) < 0.001 and (prev_pose_phantom.orientation.y - act_pose_phantom.position.y) < 0.001 and (prev_pose_phantom.orientation.z - act_pose_phantom.position.z) < 0.001:
                    change = False
                    
        else:
            wrench.wrench.force.x = (0.0 - act_pose_phantom.position.x)
            wrench.wrench.force.y = (0.0 - act_pose_phantom.position.y)
            wrench.wrench.force.z = (0.0 - act_pose_phantom.position.z)
            
            if (0.0 - act_pose_phantom.position.x) < 0.001 and (0.0 - act_pose_phantom.position.y) < 0.001 and (0.0 - act_pose_phantom.position.z) < 0.001:
                change = False
        pub_f.publish(wrench)
        
        
    else:
        pub.publish(pose)
        
    r.sleep()


frame_ = Image()
frame2_ = Image()
bridge = CvBridge()
a = False
f = False
def camera_cb(data):
    global frame_, a, bridge
    frame_ = bridge.imgmsg_to_cv2(data)
    a = True
    
def camera_cb2(data):
    global bridge, frame2_, f
    frame2_ = bridge.imgmsg_to_cv2(data)
    
    f = True

rospy.Subscriber("/robot_camera/image_raw", Image, camera_cb)
rospy.Subscriber("/robot_camera2/image_raw", Image, camera_cb2)


while a==False or f == False:
    pass


dpg.create_context()
dpg.create_viewport(title='Teleoperación', width=1460, height=1200)
dpg.setup_dearpygui()


data = np.flip(frame_, 2)  # because the camera data comes in as BGR and we need RGB
data = data.ravel()  # flatten camera data to a 1 d stricture
data = np.asfarray(data, dtype='f')  # change data type to 32bit floats
texture_data = np.true_divide(data, 255.0)  # normalize image data to prepare for GPU

data2 = np.flip(frame2_, 2)  # because the camera data comes in as BGR and we need RGB
data2 = data2.ravel()  # flatten camera data to a 1 d stricture
data2 = np.asfarray(data2, dtype='f')  # change data type to 32bit floats
texture_data2 = np.true_divide(data2, 255.0)  # normalize image data to prepare for GPU


with dpg.texture_registry(show=False):
    dpg.add_raw_texture(frame_.shape[1], frame_.shape[0], texture_data, tag="texture_tag", format=dpg.mvFormat_Float_rgb)
    dpg.add_raw_texture(frame2_.shape[1], frame2_.shape[0], texture_data2, tag="texture_tag2", format=dpg.mvFormat_Float_rgb)

    
with dpg.window(label="Cámara UR5", pos = [800,0]):
    dpg.add_image("texture_tag")
    
 
with dpg.window(label="Cámara UR52", pos = [800,500]):
    dpg.add_image("texture_tag2")
    

dpg.show_viewport()
while dpg.is_dearpygui_running():
    data = np.flip(frame_, 2)
    data = data.ravel()
    data = np.asfarray(data, dtype='f')
    texture_data = np.true_divide(data, 255.0)
    dpg.set_value("texture_tag", texture_data)
    
    data2 = np.flip(frame2_, 2)
    data2 = data2.ravel()
    data2 = np.asfarray(data2, dtype='f')
    texture_data2 = np.true_divide(data2, 255.0)
    dpg.set_value("texture_tag2", texture_data2)

    dpg.render_dearpygui_frame()
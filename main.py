import roslaunch
import rospy
import subprocess
import dearpygui.dearpygui as dpg
from math import pi
from std_msgs.msg import Int32, String
import os


# Start roscore process on the same terminal
subprocess.Popen('roscore')
rospy.sleep(1)

# GUI node
rospy.init_node("gui_node")

# Initialize DearPyGUI
dpg.create_context()
dpg.create_viewport(title='GUI', width=400, height=400)

# --------- Default variables for launch arguments -----------
names = ["ur5_2", "ur5_1"]                                          # Robot names
positions = [["0.0","0.0","0.0"], ["0.0", "0.7", "0.0"]]            # Robot positions
positions_obj = [["1.0", "1.0", "1.0"], ["0.0", "0.0", "0.0"]]      # Objects positions
n = "1"                                                             # Number of robots
gripp = ["3f", "2f_140"]                                            # Gripper options
pybullet = True                                                     # Pybullet flag

# Object names shown in the GUI
spawnables_show = ("Masterchef can", "Cracker box", "Sugar box", "Tomatosoup can", "Mustard bottle", "Tuna fish can", "Pudding box", "Gelatin box", "Potted meat can", "Banana", "Strawberry", "Apple", "Lemon", "Peach", "Pear", "Orange", "Plum", "Bleach cleanser")
    
# Object code names
spawnables = ("002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "006_mustard_bottle", "007_tuna_fish_can", "008_pudding_box", "009_gelatin_box", "010_potted_meat_can", "011_banana", "012_strawberry", "013_apple", "014_lemon", "015_peach", "016_pear", "017_orange", "018_plum", "021_bleach_cleanser")

# Object selected to spawn
spawn_name = "002_master_chef_can"

# Flags
is_launch = False       # Flag to launch the world
is_stop_sim = False     # Flag to stop the simulation
is_spawn = False        # Flag to spawn a model

change1 = True          # Variable to detect if the robot's 1 state is changing
change2 = True          # Variable to detect if the robot's 2 state is changing

calibration_file1 = "/home/daniel/Desktop/calibration.yaml"  # Route of the calibration file for the UR5e 1
calibration_file2 = "/home/daniel/Desktop/calibration.yaml"  # Route of the calibration file for the UR5e 2
ip1 = "172.18.34.201"   # IP directions for the robots
ip2 = "172.18.34.200"

# List of possibles states
state_list = ["position", "rotation", "gripper"]

# Tag names of the tutorial elements
tut_tags = ["tut_sim",  "tut_real", "tut_act", "tut_deact", "tut_act2", "tut_deact2", 
"tut_n", "tut_grip1", "tut_spa1", "tut_grip2", "tut_spa2", "tut_name1", "tut_name2", "tut_launch",
"tut_ph_conf", "tut_ph_calib", "tut_spawn_name", "tut_add_obj", "tut_stop_sim", 
"tut_x_slider", "tut_y_slider", "tut_z_slider", "tut_roll_slider", "tut_pitch_slider", 
"tut_yaw_slider", "tut_sim_op", "tut_add_int", "tut_calib1", "tut_calib2", "tut_ip1", "tut_ip2", "tut_calib_ur51", "tut_calib_ur52"]

# Simulation Options
sim_options = ["Gazebo", "Pybullet"]
real = False

# Interface flag
interf = False
is_interf = False



# -------------------- GUI callbacks -------------------------
# Changes the overviw between real and simulation
def change2simulation(sender, app_data, user_data):
    global real

    dpg.configure_item("Project", label="Simulation")
    dpg.configure_item("select_sim", show=True)
    dpg.configure_item("spawn_pos1", show=True)
    dpg.configure_item("spawn_pos2", show=True)

    dpg.configure_item("calibration_file1", show=False)
    dpg.configure_item("calibration_file2", show=False)
    dpg.configure_item("ip1", show=False)
    dpg.configure_item("ip2", show=False)
    dpg.configure_item("calib_ur51", show=False)
    dpg.configure_item("calib_ur52", show=False)
    real = False

    

def change2real(sender, app_data, user_data):
    global real
    
    dpg.configure_item("Project", label="Real")
    dpg.configure_item("select_sim", show=False)
    dpg.configure_item("spawn_pos1", show=False)
    dpg.configure_item("spawn_pos2", show=False)

    dpg.configure_item("calibration_file1", show=True)
    dpg.configure_item("calibration_file2", show=True) 
    dpg.configure_item("ip1", show=True)
    dpg.configure_item("ip2", show=True)
    dpg.configure_item("calib_ur51", show=True)
    dpg.configure_item("calib_ur52", show=True)
    real = True


# Activate and deactivate tutorial messages
def activate_tut(sender, app_data, user_data):
    global tut_tags

    for i in tut_tags:
        dpg.configure_item(i, show=True)

def deactivate_tut(sender, app_data, user_data):
    global tut_tags

    for i in tut_tags:
        dpg.configure_item(i, show=False)
    

# Number of robots selection
def n_robots(sender, app_data, user_data):
    global n

    n = dpg.get_value(sender)

    if n == "0":
        dpg.configure_item("ur51", show=False)
        dpg.configure_item("ur52", show=False)

        dpg.configure_item("change1", show=False)
        dpg.configure_item("change2", show=False)
    elif n == "1":
        dpg.configure_item("ur51", show=True)
        dpg.configure_item("ur52", show=False)

        dpg.configure_item("change1", show=True)
        dpg.configure_item("change2", show=False)
    else:
        dpg.configure_item("ur51", show=True)
        dpg.configure_item("ur52", show=True)

        dpg.configure_item("change1", show=True)
        dpg.configure_item("change2", show=True)

# Calibration file route
def calib1_cb(sender, app_data, user_data):
    global calibration_file1

    calibration_file1 = dpg.get_value(sender)

def calib2_cb(sender, app_data, user_data):
    global calibration_file2

    calibration_file2 = dpg.get_value(sender)

# Gripper option and robot name - 1
def grip1_cb(sender, app_data, user_data):
    global gripp

    gripp[0] = dpg.get_value(sender)

def name1_cb(sender, app_data, user_data):
    global names

    names[0] = dpg.get_value(sender)


# Gripper option and robot name - 2
def name2_cb(sender, app_data, user_data):
    global names

    names[1] = dpg.get_value(sender)

def grip2_cb(sender, app_data, user_data):
    global gripp

    gripp[1] = dpg.get_value(sender)


# Robot positions
def spawn_ur51(sender, app_data, user_data):
    global positions
    
    pos = dpg.get_value(sender)
    positions[0] = [str(pos[0]), str(pos[1]), str(pos[2])]

def spawn_ur52(sender, app_data, user_data):
    global positions
    
    pos = dpg.get_value(sender)
    positions[1] = [str(pos[0]), str(pos[1]), str(pos[2])]

# Select an IP direction for both robots
def ip1_cb(sender, app_data, user_data):
    global ip1

    ip1 = dpg.get_value(sender)

def ip2_cb(sender, app_data, user_data):
    global ip2

    ip2 = dpg.get_value(sender)


# Select to launch simulation with Pybullet
def sim_op(sender, app_data, user_data):
    global pybullet, sim_options

    option = dpg.get_value(sender)

    if option == sim_options[0]:
        pybullet = False

    elif option == sim_options[1]:
        pybullet = True


# Calibrate the robots
def calib_ur5_1_cb(sender, app_data, user_data):
    global calibration_file1, ip1

    # Initializes the roslaunch object with the selected arguments
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    calib_str = "target_filename:=" + calibration_file1
    ip_str = "robot_ip:=" + ip1

    cli_args = ['src/Universal_Robots_ROS_Driver/ur_calibration/launch/calibration_correction.launch', ip_str, calib_str]

    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    # Executes the launch
    launch.start()


def calib_ur5_2_cb(sender, app_data, user_data):
    global calibration_file2, ip2

    # Initializes the roslaunch object with the selected arguments
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    calib_str = "target_filename:=" + calibration_file2
    ip_str = "robot_ip:=" + ip2

    cli_args = ['src/Universal_Robots_ROS_Driver/ur_calibration/launch/calibration_correction.launch', ip_str, calib_str]

    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

    # Executes the launch
    launch.start()

# Configure and calibration of the Phantom buttons
def conf_ph(sender, app_data, user_data):
    subprocess.Popen('./Touch_Setup')

def calib_ph(sender, app_data, user_data):
    subprocess.Popen('./Touch_Diagnostic')


# Launch simulation button and configures the interface buttons
def launch_sim(sender, app_data, user_data):
    global is_launch, names, n, pybullet, real

    is_launch = True
    dpg.configure_item("conf_w", show=False)
    dpg.configure_item("exec_w", show=True)

    if not real:
        dpg.configure_item("exec_w", label="Simulation Going")
        
        dpg.configure_item("spawnables_obj", show=True)
        dpg.configure_item("pos_obj_text", show=True)
        dpg.configure_item("or_obj_text", show=True)
        dpg.configure_item("x", show=True)
        dpg.configure_item("y", show=True)
        dpg.configure_item("z", show=True)
        dpg.configure_item("roll", show=True)
        dpg.configure_item("pitch", show=True)
        dpg.configure_item("yaw", show=True)
        dpg.configure_item("add_obj", show=True)

        # Configure interface button
        if not pybullet:
            dpg.configure_item("add_interface", label="Activate video")

            dpg.configure_item("tut_text_interf", default_value="Click to configure video")


            dpg.configure_item("tut_add_int", show = True)
            dpg.configure_item("add_interface", show=True)
        
        else:
            dpg.configure_item("tut_add_int", show = False)
            dpg.configure_item("add_interface", show=False)

    else:
        dpg.configure_item("exec_w", label="Project Going")
        dpg.configure_item("spawnables_obj", show=False)
        dpg.configure_item("pos_obj_text", show=False)
        dpg.configure_item("or_obj_text", show=False)
        dpg.configure_item("x", show=False)
        dpg.configure_item("y", show=False)
        dpg.configure_item("z", show=False)
        dpg.configure_item("roll", show=False)
        dpg.configure_item("pitch", show=False)
        dpg.configure_item("yaw", show=False)
        dpg.configure_item("add_obj", show=False)
        
        

# --------------- Simulation callbacks -------------

# When new object is selected, changes the GUIs tags
def spawn_names_cb(sender, app_data, user_data):
    global spawnables, spawn_name, spawnables_show

    obj = dpg.get_value(sender)

    i = spawnables_show.index(obj)

    spawn_name = spawnables[i]

    dpg.configure_item("pos_obj_text", default_value=spawnables_show[i] + " Position")
    dpg.configure_item("or_obj_text", default_value=obj + " Orientation")


# Gets the positions and orientation by coordinates
def spawn_obj_x(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[0][0] = str(pos)

def spawn_obj_y(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[0][1] = str(pos)

def spawn_obj_z(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[0][2] = str(pos)

def spawn_obj_roll(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[1][0] = str(pos)

def spawn_obj_pitch(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[1][1] = str(pos)

def spawn_obj_yaw(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[1][2] = str(pos)

# Button for object addition to the simulation
def add_obj_cb(sender, app_data, user_data):
    global is_spawn

    is_spawn = True


# When stopping the simulation, change overview
def stop_sim_cb(sender, app_data, user_data):
    global is_stop_sim

    is_stop_sim = True

    dpg.configure_item("conf_w", show=True)
    dpg.configure_item("exec_w", show=False)


# Callbacks to show the state of the robots
def change_cb1(data):
    global state_list
    global names

    msg = ["NOT A STATE",
           "MOVING: " + names[0] + " in: POSITION mode", 
           "MOVING: " + names[0] + " in: ORIENTATION mode",
           "MOVING: " + names[0] + " in: GRIPPER mode",
           "CHANGE: " + names[0] + " to: GRIPPER coordinates ...",
           "CHANGE: " + names[0] + " to: ORIENTATION coordinates ...",
           "CHANGE: " + names[0] + " to: POSITION coordinates ..."]

    if n == "1" or n == "2":
        dpg.configure_item("change1", default_value = msg[data.data])

def change_cb2(data):
    global state_list
    global names

    msg = ["NOT A STATE",
           "MOVING: " + names[1] + " in: POSITION mode", 
           "MOVING: " + names[1] + " in: ORIENTATION mode",
           "MOVING: " + names[1] + " in: GRIPPER mode",
           "CHANGE: " + names[1] + " to: GRIPPER coordinates ...",
           "CHANGE: " + names[1] + " to: ORIENTATION coordinates ...",
           "CHANGE: " + names[1] + " to: POSITION coordinates ..."]

    if n == "2":
        dpg.configure_item("change2", default_value = msg[data.data])

def add_interf(sender, app_data, user_data):
    global interf, names, is_interf

    interf = not interf

    is_interf = True

    if not interf:
        dpg.configure_item("tut_text_interf", default_value="Click to activate video")

        dpg.configure_item("add_interface", label="Activate video")

    else:
        dpg.configure_item("tut_text_interf", default_value="Click to deactivate video")

        dpg.configure_item("add_interface", label="Deactivate video")


    

# --------------------------------- GUI ---------------------------------
with dpg.window(label="Configuration", tag="conf_w", width=400, height=400):

    # Menue Bar
    with dpg.menu_bar():
        # Select between simulation and real mode
        with dpg.menu(label="Mode"):
            dpg.add_menu_item(label="Simulation", tag="Simulation_mode", callback=change2simulation)
            with dpg.tooltip(dpg.last_item(), tag="tut_sim"):
                dpg.add_text("Click to access simulation configuration")

            dpg.add_menu_item(label="Real", tag="Real_mode", callback=change2real)
            with dpg.tooltip(dpg.last_item(), tag="tut_real"):
                dpg.add_text("Click to access real robot configuration")

        # Activate or deactivate tutorial messages
        with dpg.menu(label="Tutorial"):
            dpg.add_menu_item(label="Activate", callback=activate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_act"):
                dpg.add_text("Clcik to activate tutorials")

            dpg.add_menu_item(label="Deactivate", callback=deactivate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_deact"):
                dpg.add_text("Click to deactivate tutorials")

    # Simulation header
    with dpg.collapsing_header(label="Simulation", tag="Project", show=True, default_open=True):
        
        items = ("0", "1", "2")     # Number of robots

        # Number of robots selector
        dpg.add_combo(label="Number of Robots",indent=8,  default_value=n, items=items, width=50, callback=n_robots)
        with dpg.tooltip(dpg.last_item(), tag="tut_n"):
            dpg.add_text("Select the number of robots to spawn")


        # ------ Robot 1 options -------
        with dpg.tree_node(label="UR5 1", indent= 15,tag="ur51", default_open=True, show=(n=="1" or n=="2")):
            
            items = ("none","2f_140", "3f")     # Gripper options
            
            # Gripper option selector
            dpg.add_combo(label="Gripper", default_value=gripp[0], items=items, width=90, callback=grip1_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_grip1"):
                dpg.add_text("Select which gripper attach to the UR5 1")

            # Position selector
            dpg.add_input_doublex(label="Spawn Position", tag="spawn_pos1", width=160, size=3, default_value=[float(positions[0][0]),float(positions[0][1]),float(positions[0][2])], callback=spawn_ur51, format='%.2f')
            with dpg.tooltip(dpg.last_item(), tag="tut_spa1"):
                dpg.add_text("Select XYZ position for the UR5 1")

            dpg.add_input_text(label="IP", tag="ip1", default_value=ip1, callback=ip1_cb, width=110, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_ip1"):
                dpg.add_text("Select an IP for the UR5 1")

            # Robot 1 name
            dpg.add_input_text(label="Robot Name", tag="name1", default_value=names[0], callback=name1_cb, width=100)
            with dpg.tooltip(dpg.last_item(), tag="tut_name1"):
                dpg.add_text("Select name for the UR5 1")

            # Select calibration directory
            dpg.add_input_text(label="Calib. file", tag="calibration_file1", width=250, default_value=calibration_file1, callback=calib1_cb, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_calib1"):
                dpg.add_text("Select a calibration file for UR5e 1")

            # UR5e calibration
            dpg.add_button(label="Calibrate UR5e 1", tag="calib_ur51", callback=calib_ur5_1_cb, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_calib_ur51"):
                dpg.add_text("Click to calibrate UR5e 1")

            dpg.add_separator()


        # ------ Robot 2 options -------
        with dpg.tree_node(label="UR5 2", indent=15, tag="ur52", default_open=True, show=(n == "2")):
            
            items = ("none","2f_140", "3f")     # Gripper options

            # Gripper option selector
            dpg.add_combo(label="Gripper", default_value=gripp[1], items=items, width=90, callback=grip2_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_grip2"):
                dpg.add_text("Select which gripper attach to the UR5 2")

            # Postion selector
            dpg.add_input_doublex(label="Spawn Position", tag="spawn_pos2", width=160, size=3, default_value=[float(positions[1][0]), float(positions[1][1]), float(positions[1][2])], callback=spawn_ur52, format='%.2f')
            with dpg.tooltip(dpg.last_item(), tag="tut_spa2"):
                dpg.add_text("Select XYZ position for the UR5 2")

            dpg.add_input_text(label="IP", tag="ip2", default_value=ip2, callback=ip2_cb, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_ip2"):
                dpg.add_text("Select an IP for the UR5 2")

            # Robot 2 name
            dpg.add_input_text(label="Robot Name", tag="name2", default_value=names[1], callback=name2_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_name2"):
                dpg.add_text("Select name for the UR5 2")

            # Select calibration directory
            dpg.add_input_text(label="Calib. file", tag="calibration_file2", default_value=calibration_file2, callback=calib2_cb, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_calib2"):
                dpg.add_text("Select a calibration file for UR5e 2")

            # UR5e calibration
            dpg.add_button(label="Calibrate UR5e 2", tag="calib_ur52", callback=calib_ur5_2_cb, show=False)
            with dpg.tooltip(dpg.last_item(), tag="tut_calib_ur52"):
                dpg.add_text("Click to calibrate UR5e 2")

            dpg.add_separator()


        # Simulator selector
        dpg.add_combo(label="Select Simulator", tag="select_sim", default_value=sim_options[1], items=sim_options, width=90, callback=sim_op)
        with dpg.tooltip(dpg.last_item(), tag="tut_sim_op"):
            dpg.add_text("Click to select Pybullet simulator")
        
        # Launch button
        dpg.add_button(label="Launch Project", tag="launch_sim", callback=launch_sim)
        with dpg.tooltip(dpg.last_item(), tag="tut_launch"):
            dpg.add_text("Clik to launch the project")


    # Phantom configuration and calibration buttons
    dpg.add_button(label="Configure Phantom", tag="conf_ph1", callback=conf_ph)
    with dpg.tooltip(dpg.last_item(), tag="tut_ph_conf"):
        dpg.add_text("Click to configure the Phantom devices")

    dpg.add_button(label="Calibrate Phantom", tag="calib_ph1", callback=calib_ph)
    with dpg.tooltip(dpg.last_item(), tag="tut_ph_calib"):
        dpg.add_text("Click to calibrate the Phantom devices")


# ---------------- Project going GUI -------------
with dpg.window(label="Simulation Going", show=False, tag="exec_w", width=400, height=400):
    
    # Menu with tutorial options
    with dpg.menu_bar():
        with dpg.menu(label="Tutorial"):
            dpg.add_menu_item(label="Activate", callback=activate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_act2"):
                dpg.add_text("Clcik to activate tutorials")

            dpg.add_menu_item(label="Deactivate", callback=deactivate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_deact2"):
                dpg.add_text("Click to deactivate tutorials")
    
    # Select objects to spawn
    dpg.add_combo(label="Spawnables Objects", tag="spawnables_obj",indent=8,  default_value="Masterchef can", items=spawnables_show, width=160, callback=spawn_names_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_spawn_name"):
        dpg.add_text("Select an object to spawn")


    dpg.add_separator()


    # Select the object position and orientation in the world
    dpg.add_text(tag="pos_obj_text", default_value="Master Chef Can Position")

    dpg.add_slider_double(label="X", tag="x", default_value=0.0, min_value=-1, max_value=1, callback=spawn_obj_x)
    with dpg.tooltip(dpg.last_item(), tag="tut_x_slider"):
        dpg.add_text("Choose desired X position")

    dpg.add_slider_double(label="y", tag="y", default_value=0.0, min_value=-1, max_value=1, callback=spawn_obj_y)
    with dpg.tooltip(dpg.last_item(), tag="tut_y_slider"):
        dpg.add_text("Choose desired y position")

    dpg.add_slider_double(label="Z", tag="z", default_value=0.0, min_value=-1, max_value=1, callback=spawn_obj_z)
    with dpg.tooltip(dpg.last_item(), tag="tut_z_slider"):
        dpg.add_text("Choose desired Z position")


    dpg.add_text(tag="or_obj_text", default_value="Macter Chef Can Orientation")

    dpg.add_slider_double(label="Roll", tag="roll", default_value=0.0, min_value=-pi, max_value=pi, callback=spawn_obj_roll)
    with dpg.tooltip(dpg.last_item(), tag="tut_roll_slider"):
        dpg.add_text("Choose desired ROLL orientation")

    dpg.add_slider_double(label="Pitch", tag="pitch", default_value=0.0, min_value=-pi, max_value=pi, callback=spawn_obj_pitch)
    with dpg.tooltip(dpg.last_item(), tag="tut_pitch_slider"):
        dpg.add_text("Choose desired pitch orientation")
        
    dpg.add_slider_double(label="Yaw", tag="yaw", default_value=0.0, min_value=-pi, max_value=pi, callback=spawn_obj_yaw)
    with dpg.tooltip(dpg.last_item(), tag="tut_yaw_slider"):
        dpg.add_text("Choose desired YAW orientation")


    dpg.add_separator()

    # Add object button
    dpg.add_button(label="Add Object", tag="add_obj", callback=add_obj_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_add_obj"):
        dpg.add_text("Click to add the object to the simulation")


    dpg.add_separator()

    # Activate video interface
    dpg.add_button(label="Activate Robot video", tag="add_interface", show = False, callback=add_interf)
    with dpg.tooltip(dpg.last_item(), tag="tut_add_int"):
        dpg.add_text("Click to add the robot interface", tag="tut_text_interf")

    dpg.add_separator()

    # Stop simulation
    dpg.add_button(label="Stop Simulation", tag="stop_sim", callback=stop_sim_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_stop_sim"):
        dpg.add_text("Click to stop the simulation")

    dpg.add_separator()

    # State messages
    dpg.add_text(tag="change1", default_value="Moving Phantom 1 to position origin ...", show = (n == "1" or n == "2"))
    dpg.add_text(tag="change2", default_value="Moving Phantom 2 to position origin ...", show = n == "2")

dpg.setup_dearpygui()
dpg.show_viewport()



# ---- Main ----
if __name__ == "__main__":

    # Generate an ID for the launch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Builds a roslacunh parent object, so it can be called inside the loop
    origin1 = 'origin1:=-x ' + positions[0][0] + ' -y ' + positions[0][1] + ' -z ' + positions[0][2]
    origin2 = 'origin2:=-x ' + positions[1][0] + ' -y ' + positions[1][1] + ' -z ' + positions[1][2]

    name1 = 'name1:=' + names[0]
    name2 = 'name2:=' + names[1]
    cli_args = ['src/universal_robot/ur_e_gazebo/launch/ur5_2.launch', 'number:=' + n,'grip1:=' + gripp[0],'grip2:=' + gripp[1], origin1, origin2, name1, name2]

    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)



    # Generate an ID for the interface launch
    uuid_interf = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid_interf)


    cli_args_interf = ['src/nodes/launch/interf.launch', "name1:=" + names[0], "name2:=" + names[1],"number:=" + n]

    roslaunch_args_interf = cli_args_interf[1:]
    roslaunch_file_interf = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_interf)[0], roslaunch_args_interf)]

    launch_interf = roslaunch.parent.ROSLaunchParent(uuid_interf, roslaunch_file_interf)


    # ---------- GUI loop --------
    while dpg.is_dearpygui_running():
        
        # If there is a launch ...
        if is_launch:

            # ... sets up the inital state message
            aux = Int32()
            aux.data = -1
            change_cb1(aux)

            if n == "2":
                change_cb2(aux)

            # Initializes the roslaunch object with the selected arguments
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            origin1 = 'origin1:=-x ' + positions[0][0] + ' -y ' + positions[0][1] + ' -z ' + positions[0][2]
            origin2 = 'origin2:=-x ' + positions[1][0] + ' -y ' + positions[1][1] + ' -z ' + positions[1][2]

            name1 = 'name1:=' + names[0]
            name2 = 'name2:=' + names[1]

            py = "sim_gaz:=true"
            if pybullet:
                py = "sim_gaz:=false"


            real_str = "real:=false"
            if real:
                real_str = "real:=true"

            calib_str1 = "calibration_file1:=" + calibration_file1
            calib_str2 = "calibration_file2:=" + calibration_file2
            ip_str1 = "ip1:=" + ip1
            ip_str2 = "ip2:=" + ip2

            cli_args = ['src/universal_robot/ur_e_gazebo/launch/ur5_2.launch', 'number:=' + n,'grip1:=' + gripp[0],
                        'grip2:=' + gripp[1], origin1, origin2, name1, name2, py, real_str, calib_str1, calib_str2, ip_str1, ip_str2]

            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

            # Executes the launch
            launch.start()
            is_launch = False

            # Depending on the number of robot, it subscribes to the change topics
            if n == "1":
                rospy.Subscriber("/" + names[0] + "/change", Int32, change_cb1)
            
            elif n == "2":
                rospy.Subscriber("/" + names[0] + "/change", Int32, change_cb1)
                rospy.Subscriber("/" + names[1] + "/change", Int32, change_cb2)


        # If there has been a stop, shutdowns the simulation
        if is_stop_sim:
            launch.shutdown()
            
            launch_interf.shutdown()

            is_stop_sim = False


        # If there is an object to be spawn ...
        if is_spawn:

            # If the simulation is running in Gazebo ...
            if not pybullet:

                # ... calls the launch for the objects
                uuid_obj = roslaunch.rlutil.get_or_generate_uuid(None, False)

                origin_obj = 'origin:=-x' + positions_obj[0][0] + ' -y ' + positions_obj[0][1] + ' -z ' + positions_obj[0][2] + ' -R ' + positions_obj[1][0] + ' -P ' + positions_obj[1][1] + ' -Y ' + positions_obj[1][2]
                object_name = 'object_name:=' + spawn_name

                cli_args_obj = ['src/objects_models/launch/object_model.launch', origin_obj, object_name]
                roslaunch_args_obj = cli_args_obj[1:]
                roslaunch_file_obj = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_obj)[0], roslaunch_args_obj)]

                parent_obj = roslaunch.parent.ROSLaunchParent(uuid_obj, roslaunch_file_obj)

                parent_obj.start()
            

            # If the simulation is running in PyBullet ...
            else:
                # ... publihses in a topic with the name of the desired object
                pyb_sp_obj = (rospy.Publisher("/object_spawn", String, queue_size=10))
                spawn_msg = String()

                # Gets the actual directory
                dir_path = os.path.dirname(os.path.realpath(__file__))
                
                spawn_msg.data = spawn_name + ' ' + positions_obj[0][0] + ' ' + positions_obj[0][1] + ' ' + positions_obj[0][2] + ' ' + positions_obj[1][0] + ' ' + positions_obj[1][1] + ' ' + positions_obj[1][2] + ' ' + dir_path

                pyb_sp_obj.publish(spawn_msg)
            
            is_spawn = False
        
        # If the interface has to be published ...
        if is_interf:
            # If the interface is shutdown, it launches it
            if interf:
                cli_args_interf = ['src/nodes/launch/interf.launch', "name1:=" + names[0], "name2:=" + names[1],"number:=" + n]

                roslaunch_args_interf = cli_args_interf[1:]
                roslaunch_file_interf = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_interf)[0], roslaunch_args_interf)]

                launch_interf = roslaunch.parent.ROSLaunchParent(uuid_interf, roslaunch_file_interf)

                # Executes the launch
                launch_interf.start()
                is_interf = False

            # If the interface is on, it shuts it down
            else:
                launch_interf.shutdown()
                is_interf = False


        # Renderizes the window
        dpg.render_dearpygui_frame()
    
    # When the program ends, kills rosmaster and closes all windows
    subprocess.run(["gnome-terminal","--", "sh", "-c","killall -9 rosmaster"])
    rospy.sleep(2)
    
    dpg.destroy_context()


    
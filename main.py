
import roslaunch
import rospy
import subprocess
import dearpygui.dearpygui as dpg

subprocess.Popen('roscore')
rospy.sleep(1)

dpg.create_context()
dpg.create_viewport(title='GUI', width=400, height=400)

names = ["ur5_1", "ur5_2"]
positions = [["-0.2","0.8","1.02"], ["-0.2", "0.1", "1.02"]]
positions_obj = [["1.0", "1.0", "1.0"], ["0.0", "0.0", "0.0"]]
n = "2"
gripp = ["2f_140", "3f"]

spawnables_show = ("Masterchef can", "Cracker box", "Sugar box", "Tomatosoup can", "Mustard bottle", "Tuna fish can", "Pudding box", "Gelatin box", "Potted meat can", "Banana", "Strawberry", "Apple", "Lemon", "Peach", "Pear", "Orange", "Plum", "Bleach cleanser")
    
spawnables = ("002_master_chef_can", "003_cracker_box", "004_sugar_box", "005_tomato_soup_can", "006_mustard_bottle", "007_tuna_fish_can", "008_pudding_box", "009_gelatin_box", "010_potted_meat_can", "011_banana", "012_strawberry", "013_apple", "014_lemon", "015_peach", "016_pear", "017_orange", "018_plum", "021_bleach_cleanser")

spawn_name = "002_master_chef_can"

is_launch = False
is_stop_sim = False
is_spawn = False

def change2simulation(sender, app_data, user_data):
    # TODO: hacer el cambio de visibilidad
    dpg.configure_item("Simulation", show=True)
    dpg.configure_item("Real", show=False)

def change2real(sender, app_data, user_data):
    dpg.configure_item("Real", show=True)
    dpg.configure_item("Simulation", show=False)

def activate_tut(sender, app_data, user_data):
    dpg.configure_item("tut_sim", show=True)
    dpg.configure_item("tut_real", show=True)
    dpg.configure_item("tut_act", show=True)
    dpg.configure_item("tut_deact", show=True)
    dpg.configure_item("tut_act2", show=True)
    dpg.configure_item("tut_deact2", show=True)
    dpg.configure_item("tut_n", show=True)
    dpg.configure_item("tut_grip1", show=True)
    dpg.configure_item("tut_spa1", show=True)
    dpg.configure_item("tut_grip2", show=True)
    dpg.configure_item("tut_spa2", show=True)
    dpg.configure_item("tut_ph_conf", show=True)
    dpg.configure_item("tut_ph_calib", show=True)
    dpg.configure_item("tut_spawn_name", show=True)
    dpg.configure_item("tut_spa_pos_obj", show=True)
    dpg.configure_item("tut_spa_or_obj", show=True)
    dpg.configure_item("tut_add_obj", show=True)
    dpg.configure_item("tut_stop_sim", show=True)


def deactivate_tut(sender, app_data, user_data):
    dpg.configure_item("tut_sim", show=False)
    dpg.configure_item("tut_real", show=False)
    dpg.configure_item("tut_act", show=False)
    dpg.configure_item("tut_deact", show=False)
    dpg.configure_item("tut_act2", show=False)
    dpg.configure_item("tut_deact2", show=False)
    dpg.configure_item("tut_n", show=False)
    dpg.configure_item("tut_grip1", show=False)
    dpg.configure_item("tut_spa1", show=False)
    dpg.configure_item("tut_grip2", show=False)
    dpg.configure_item("tut_spa2", show=False)
    dpg.configure_item("tut_ph_conf", show=False)
    dpg.configure_item("tut_ph_calib", show=False)
    dpg.configure_item("tut_spawn_name", show=False)
    dpg.configure_item("tut_spa_pos_obj", show=False)
    dpg.configure_item("tut_spa_or_obj", show=False)
    dpg.configure_item("tut_add_obj", show=False)
    dpg.configure_item("tut_stop_sim", show=False)


def n_robots(sender, app_data, user_data):
    global n

    n = dpg.get_value(sender)

    if n == "0":
        dpg.configure_item("ur51", show=False)
        dpg.configure_item("ur52", show=False)
    elif n == "1":
        dpg.configure_item("ur51", show=True)
        dpg.configure_item("ur52", show=False)
    else:
        dpg.configure_item("ur51", show=True)
        dpg.configure_item("ur52", show=True)

def grip1_cb(sender, app_data, user_data):
    global gripp

    gripp[0] = dpg.get_value(sender)

def name1_cb(sender, app_data, user_data):
    global names

    names[0] = dpg.get_value(sender)

def name2_cb(sender, app_data, user_data):
    global names

    names[1] = dpg.get_value(sender)

def spawn_ur51(sender, app_data, user_data):
    global positions
    
    pos = dpg.get_value(sender)
    positions[0] = [str(pos[0]), str(pos[1]), str(pos[2])]

def grip2_cb(sender, app_data, user_data):
    global gripp

    gripp[1] = dpg.get_value(sender)

def spawn_ur52(sender, app_data, user_data):
    global positions
    
    pos = dpg.get_value(sender)
    positions[1] = [str(pos[0]), str(pos[1]), str(pos[2])]

def conf_ph(sender, app_data, user_data):
    subprocess.Popen('./Touch_Setup')

def calib_ph(sender, app_data, user_data):
    subprocess.Popen('./Touch_Diagnostic')

def launch_sim(sender, app_data, user_data):
    global is_launch

    is_launch = True
    dpg.configure_item("conf_w", show=False)
    dpg.configure_item("exec_w", show=True)

def spawn_names_cb(sender, app_data, user_data):
    global spawnables, spawn_name, spawnables_show

    obj = dpg.get_value(sender)

    i = spawnables_show.index(obj)

    spawn_name = spawnables[i]

def spawn_pos_obj(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[0] = [str(pos[0]), str(pos[1]), str(pos[2])]

def spawn_or_obj(sender, app_data, user_data):
    global positions_obj
    
    pos = dpg.get_value(sender)
    positions_obj[1] = [str(pos[0]), str(pos[1]), str(pos[2])]

def add_obj_cb(sender, app_data, user_data):
    global is_spawn

    is_spawn = True

def stop_sim_cb(sender, app_data, user_data):
    global is_stop_sim

    is_stop_sim = True

    dpg.configure_item("conf_w", show=True)
    dpg.configure_item("exec_w", show=False)

    
    

with dpg.window(label="Configuration", tag="conf_w", width=400, height=400):
    with dpg.menu_bar():
        with dpg.menu(label="Mode"):
            dpg.add_menu_item(label="Simulation", tag="Simulation_mode", callback=change2simulation)
            with dpg.tooltip(dpg.last_item(), tag="tut_sim"):
                dpg.add_text("Click to access simulation configuration")

            dpg.add_menu_item(label="Real", tag="Real_mode", callback=change2real)
            with dpg.tooltip(dpg.last_item(), tag="tut_real"):
                dpg.add_text("Click to access real robot configuration")

        with dpg.menu(label="Tutorial"):
            dpg.add_menu_item(label="Activate", callback=activate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_act"):
                dpg.add_text("Clcik to activate tutorials")

            dpg.add_menu_item(label="Deactivate", callback=deactivate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_deact"):
                dpg.add_text("Click to deactivate tutorials")


    with dpg.collapsing_header(label="Simulation", tag="Simulation", show=True, default_open=True):
        items = ("0", "1", "2")
        dpg.add_combo(label="Number of Robots",indent=8,  default_value="2", items=items, width=50, callback=n_robots)
        with dpg.tooltip(dpg.last_item(), tag="tut_n"):
            dpg.add_text("Select the number of robots to spawn")

        with dpg.tree_node(label="UR5 1", indent= 15,tag="ur51", default_open=True):
            items = ("none","2f 140", "3f")
            dpg.add_combo(label="Gripper", default_value="2f 140", items=items, width=90, callback=grip1_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_grip1"):
                dpg.add_text("Select which gripper attach to the UR5 1")

            dpg.add_input_doublex(label="Spawn Position", width=160, size=3, default_value=[-0.2,0.8,1.02], callback=spawn_ur51, format='%.2f')
            with dpg.tooltip(dpg.last_item(), tag="tut_spa1"):
                dpg.add_text("Select XYZ position for the UR5 1")

            dpg.add_input_text(label="Robot Name", tag="name1", default_value="ur5_1", callback=name1_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_name1"):
                dpg.add_text("Select name for the UR5 1")

            dpg.add_separator()


        with dpg.tree_node(label="UR5 2", indent=15, tag="ur52", default_open=True):
            items = ("none","2f 140", "3f")
            dpg.add_combo(label="Gripper", default_value="3f", items=items, width=90, callback=grip2_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_grip2"):
                dpg.add_text("Select which gripper attach to the UR5 2")

            dpg.add_input_doublex(label="Spawn Position", width=160, size=3, default_value=[-0.2, 0.1, 1.02], callback=spawn_ur52, format='%.2f')
            with dpg.tooltip(dpg.last_item(), tag="tut_spa2"):
                dpg.add_text("Select XYZ position for the UR5 2")

            dpg.add_input_text(label="Robot Name", tag="name2", default_value="ur5_2", callback=name2_cb)
            with dpg.tooltip(dpg.last_item(), tag="tut_name2"):
                dpg.add_text("Select name for the UR5 2")

            dpg.add_separator()
        
        dpg.add_button(label="Launch Simulation", tag="launch_sim", callback=launch_sim)
        with dpg.tooltip(dpg.last_item(), tag="tut_launch"):
            dpg.add_text("Clcik to launch the simulation")

    # TODO: GUI PARA EL CASO DE MANEJO CON EL REAL
    with dpg.collapsing_header(label="Real", tag="Real",default_open=True, show=False):
        pass


    dpg.add_button(label="Configure Phantom", tag="conf_ph1", callback=conf_ph)
    with dpg.tooltip(dpg.last_item(), tag="tut_ph_conf"):
        dpg.add_text("Clcik to configure the Phantom devices")

    dpg.add_button(label="Calibrate Phantom", tag="calib_ph1", callback=calib_ph)
    with dpg.tooltip(dpg.last_item(), tag="tut_ph_calib"):
        dpg.add_text("Clcik to calibrate the Phantom devices")


with dpg.window(label="Simulation Going", show=False, tag="exec_w", width=400, height=400):
    with dpg.menu_bar():
        with dpg.menu(label="Tutorial"):
            dpg.add_menu_item(label="Activate", callback=activate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_act2"):
                dpg.add_text("Clcik to activate tutorials")

            dpg.add_menu_item(label="Deactivate", callback=deactivate_tut)
            with dpg.tooltip(dpg.last_item(), tag="tut_deact2"):
                dpg.add_text("Click to deactivate tutorials")
    
    dpg.add_combo(label="Spawnables Objects",indent=8,  default_value="Masterchef can", items=spawnables_show, width=160, callback=spawn_names_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_spawn_name"):
        dpg.add_text("Select an object to spawn")

    dpg.add_input_doublex(label="Object Spawn Position", width=160, size=3, default_value=[1,1,1], callback=spawn_pos_obj, format='%.2f')
    with dpg.tooltip(dpg.last_item(), tag="tut_spa_pos_obj"):
        dpg.add_text("Select XYZ position for the object")
    
    dpg.add_input_doublex(label="Object Spawn Rotation", width=160, size=3, default_value=[0.0,0.0,0.0], callback=spawn_or_obj, format='%.2f')
    with dpg.tooltip(dpg.last_item(), tag="tut_spa_or_obj"):
        dpg.add_text("Select RPY orientation for the object")

    dpg.add_button(label="Add Object", tag="add_obj", callback=add_obj_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_add_obj"):
        dpg.add_text("Clcik to add the object to the simulation")

    dpg.add_button(label="Stop Simulation", tag="stop_sim", callback=stop_sim_cb)
    with dpg.tooltip(dpg.last_item(), tag="tut_stop_sim"):
        dpg.add_text("Clcik to stop the simulation")
        

dpg.setup_dearpygui()
dpg.show_viewport()

if __name__ == "__main__":

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    origin1 = 'origin1:=-x ' + positions[0][0] + ' -y ' + positions[0][1] + ' -z ' + positions[0][2]
    origin2 = 'origin2:=-x ' + positions[1][0] + ' -y ' + positions[1][1] + ' -z ' + positions[1][2]

    name1 = 'name1:=' + names[0]
    name2 = 'name2:=' + names[1]
    cli_args = ['src/universal_robot/ur_e_gazebo/launch/ur5_2.launch', 'number:=' + n,'grip1:=' + gripp[0],'grip2:=' + gripp[1], origin1, origin2, name1, name2]

    roslaunch_args = cli_args[1:]
    roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)


    while dpg.is_dearpygui_running():
        if is_launch:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)

            origin1 = 'origin1:=-x ' + positions[0][0] + ' -y ' + positions[0][1] + ' -z ' + positions[0][2]
            origin2 = 'origin2:=-x ' + positions[1][0] + ' -y ' + positions[1][1] + ' -z ' + positions[1][2]

            name1 = 'name1:=' + names[0]
            name2 = 'name2:=' + names[1]
            cli_args = ['src/universal_robot/ur_e_gazebo/launch/ur5_2.launch', 'number:=' + n,'grip1:=' + gripp[0],'grip2:=' + gripp[1], origin1, origin2, name1, name2]

            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            is_launch = False

        if is_stop_sim:
            launch.shutdown()

            is_stop_sim = False

        if is_spawn:
            uuid_obj = roslaunch.rlutil.get_or_generate_uuid(None, False)

            origin_obj = 'origin:=-x' + positions_obj[0][0] + ' -y ' + positions_obj[0][1] + ' -z ' + positions_obj[0][2] + ' -R ' + positions_obj[1][0] + ' -P ' + positions_obj[1][1] + ' -Y ' + positions_obj[1][2]
            object_name = 'object_name:=' + spawn_name

            cli_args_obj = ['src/objects_models/launch/object_model.launch', origin_obj, object_name]
            roslaunch_args_obj = cli_args_obj[1:]
            roslaunch_file_obj = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_obj)[0], roslaunch_args_obj)]

            parent_obj = roslaunch.parent.ROSLaunchParent(uuid_obj, roslaunch_file_obj)

            parent_obj.start()

            is_spawn = False

        dpg.render_dearpygui_frame()

    subprocess.run(["gnome-terminal","--", "sh", "-c","killall -9 rosmaster"])
    rospy.sleep(2)
    
    dpg.destroy_context()


    

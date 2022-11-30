#! /usr/bin/python3

import moveit_commander
arm = moveit_commander.MoveGroupCommander("gripper")

arm.set_goal_tolerance(0.1)
arm.set_named_target("Closed")
arm.go()
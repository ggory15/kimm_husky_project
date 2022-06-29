#! /usr/bin/python3 

import rospy
import actionlib
import kimm_action_manager.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray
import numpy as np
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
from std_msgs.msg import Bool

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('simulation_actions_client')
        self.ns0_joint_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/joint_posture_control', kimm_action_manager.msg.JointPostureAction)
        self.ns0_joint_ctrl_client.wait_for_server()
        self.ns0_se3_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/se3_control', kimm_action_manager.msg.SE3Action)
        self.ns0_se3_ctrl_client.wait_for_server()
        self.ns0_se3_array_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/se3_array_control', kimm_action_manager.msg.SE3ArrayAction)
        self.ns0_se3_array_ctrl_client.wait_for_server()
        self.ns0_gripper_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/gripper_control', kimm_action_manager.msg.GripperAction)
        self.ns0_gripper_ctrl_client.wait_for_server()
        self.ns0_move_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/move_control', kimm_action_manager.msg.MoveAction)
        self.ns0_move_ctrl_client.wait_for_server()
        self.ns0_gravity_pub = rospy.Publisher('/ns1/kimm_action_manager/gravity_ctrl', Bool, queue_size=1)
        self.ns0_gravity = True

        self.ns1_joint_ctrl_client = actionlib.SimpleActionClient('/ns1/kimm_action_manager/joint_posture_control', kimm_action_manager.msg.JointPostureAction)
        self.ns1_joint_ctrl_client.wait_for_server()
        self.ns1_se3_ctrl_client = actionlib.SimpleActionClient('/ns1/kimm_action_manager/se3_control', kimm_action_manager.msg.SE3Action)
        self.ns1_se3_ctrl_client.wait_for_server()
        self.ns1_se3_array_ctrl_client = actionlib.SimpleActionClient('/ns1/kimm_action_manager/se3_array_control', kimm_action_manager.msg.SE3ArrayAction)
        self.ns1_se3_array_ctrl_client.wait_for_server()
        self.ns1_gripper_ctrl_client = actionlib.SimpleActionClient('/ns1/kimm_action_manager/gripper_control', kimm_action_manager.msg.GripperAction)
        self.ns1_gripper_ctrl_client.wait_for_server()
        self.ns1_move_ctrl_client = actionlib.SimpleActionClient('/ns1/kimm_action_manager/move_control', kimm_action_manager.msg.MoveAction)
        self.ns1_move_ctrl_client.wait_for_server()
        self.ns1_gravity_pub = rospy.Publisher('/ns1/kimm_action_manager/gravity_ctrl', Bool, queue_size=1)
        self.ns1_gravity = True
    
    def do_home(self, arg):
        'Go to the home position using joint posture ctrl'
        goal = kimm_action_manager.msg.JointPostureGoal
        goal.duration = 2.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -1.57/2.0])
        
        self.ns0_joint_ctrl_client.send_goal(goal)
        self.ns1_joint_ctrl_client.send_goal(goal)

        self.ns0_joint_ctrl_client.wait_for_result()
        if (self.ns0_joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")        
        self.ns1_joint_ctrl_client.wait_for_result()
        if (self.ns1_joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    
    def do_gravity(self, arg):
        'Turn On/Off Gravity Ctrl'
        if (self.ns0_gravity):
            print ("Now Gravity Mode Off")
            self.ns0_gravity = False
            self.ns0_gravity_pub.publish(False)

        else:
            print ("Now Gravity Mode On")
            self.ns0_gravity = True
            self.ns0_gravity_pub.publish(True)

        if (self.ns1_gravity):
            print ("Now Gravity Mode Off")
            self.ns1_gravity = False
            self.ns1_gravity_pub.publish(False)

        else:
            print ("Now Gravity Mode On")
            self.ns1_gravity = True
            self.ns1_gravity_pub.publish(True)

    def do_move(self, arg):
        'Move'
        goal = kimm_action_manager.msg.MoveGoal

        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.position.y = 2.0
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0 
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.258819
        goal.target_pose.pose.orientation.w = 0.9659258

        self.ns0_move_ctrl_client.send_goal(goal)
        

        if (self.ns0_move_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.position.y = -2.0
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0 
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.258819
        goal.target_pose.pose.orientation.w = 0.9659258

        self.ns1_move_ctrl_client.send_goal(goal)
        
        self.ns0_move_ctrl_client.wait_for_result()
        self.ns1_move_ctrl_client.wait_for_result()

        if (self.ns1_move_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
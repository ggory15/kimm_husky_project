#! /usr/bin/python3 

import rospy
import actionlib
import kimm_action_manager.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import numpy as np
import importlib, pkgutil
import threading
import cmd, sys, os
import copy
from std_msgs.msg import Bool
from slam_toolbox_msgs.srv import SaveMap, SerializePoseGraph, DeserializePoseGraph
import subprocess, signal, os, inspect, time


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
        self.joint_ctrl_client = actionlib.SimpleActionClient('/ns1/basic_husky_franka_controller/kimm_action_manager/joint_posture_control', kimm_action_manager.msg.JointPostureAction)
        self.joint_ctrl_client.wait_for_server()
        self.se3_ctrl_client = actionlib.SimpleActionClient('/ns1/basic_husky_franka_controller/kimm_action_manager/se3_control', kimm_action_manager.msg.SE3Action)
        self.se3_ctrl_client.wait_for_server()
        self.se3_array_ctrl_client = actionlib.SimpleActionClient('/ns1/basic_husky_franka_controller/kimm_action_manager/se3_array_control', kimm_action_manager.msg.SE3ArrayAction)
        self.se3_array_ctrl_client.wait_for_server()
        self.move_ctrl_client = actionlib.SimpleActionClient('/ns1/basic_husky_franka_controller/kimm_action_manager/move_control', kimm_action_manager.msg.MoveAction)
        self.move_ctrl_client.wait_for_server()
        self.qr_ctrl_client = actionlib.SimpleActionClient('/ns1/basic_husky_franka_controller/kimm_action_manager/qr_control', kimm_action_manager.msg.QRAction)
        self.qr_ctrl_client.wait_for_server()

        self.gravity_pub = rospy.Publisher('/ns1/basic_husky_franka_controller/kimm_action_manager/gravity_ctrl', Bool, queue_size=1)
        self.gravity = True
        self.group_name = "ns0"
        # rospy.wait_for_service('/ns1/slam_toolbox/save_map')
        # rospy.wait_for_service('/ns1/slam_toolbox/serialize_map')
        # self.initial_pose_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.InitCallback)
        # self.initial_pose_pub = rospy.Publisher(self.group_name + "/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.movebase_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.moveCallback)
        self.movebase_ns0_pub = rospy.Publisher("/ns0/husky/move_base/move_base_simple/goal", PoseStamped, queue_size=1)
        self.movebase_ns1_pub = rospy.Publisher("/ns1/husky/move_base/move_base_simple/goal", PoseStamped, queue_size=1)

    def InitCallback(self, msg):
        # self.initial_pose_pub.publish(msg)
        a=1

    def moveCallback(self, msg):
        if (self.group_name == "ns0"):
            self.movebase_ns0_pub.publish(msg)
        if (self.group_name == "ns1"):
            self.movebase_ns1_pub.publish(msg)
        
            
    def do_mapsave(self, arg):
        'Save the map'
        # save_map = rospy.ServiceProxy('/ns1/slam_toolbox/save_map', SaveMap)
        # map_name = String()
        # map_name.data = "kimm_241_map"
        # resp1 = save_map(map_name)
        # print("done saving map")
        self.savemap = subprocess.Popen("rosrun map_server map_saver -f ~/.ros/kimm_241_map", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        serialize_map = rospy.ServiceProxy('/ns1/slam_toolbox/serialize_map', SerializePoseGraph)
        map_name = "kimm_241_map"
        resp1 = serialize_map(map_name)
        print("done serialize map")

    def do_changegroup(self, arg):
        if (len(arg) > 0):
            if (int(arg) == 0):
                self.group_name = "ns0"
            elif (int(arg)==1):
                self.group_name = "ns1"
            else:
                self.group_name = "ns0"
        else:
            self.group_name = "ns0"

        print ("Current Group is")
        print (self.group_name)

    def do_home(self, arg):
        'Go to the home position using joint posture ctrl'
        goal = kimm_action_manager.msg.JointPostureGoal
        goal.duration = 5.0
        goal.target_joints = JointState()
        goal.target_joints.position = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, -1.57/2.0])
        
        self.joint_ctrl_client.send_goal(goal)
        self.joint_ctrl_client.wait_for_result()
        if (self.joint_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    
    def do_gravity(self, arg):
        'Turn On/Off Gravity Ctrl'
        if (self.gravity):
            print ("Now Gravity Mode Off")
            self.gravity = False
            self.gravity_pub.publish(False)

        else:
            print ("Now Gravity Mode On")
            self.gravity = True
            self.gravity_pub.publish(True)

    def do_qr(self, arg):
        goal = kimm_action_manager.msg.QRGoal
        goal.duration = 10.0

        goal.qr_pose = Pose()
        goal.qr_pose.position.x = 1.35
        goal.qr_pose.position.y = -0.15
        goal.qr_pose.position.z = 0.2

        goal.qr_pose.orientation.x = 0.
        goal.qr_pose.orientation.y = 0.
        goal.qr_pose.orientation.z = 0.08
        goal.qr_pose.orientation.w = 0.99

        goal.target_pose = Pose()
        goal.target_pose.position.x = 0.15
        goal.target_pose.position.y = 0
        goal.target_pose.position.z = 0.2

        goal.target_pose.orientation.x = 0.
        goal.target_pose.orientation.y = 0
        goal.target_pose.orientation.z = 0
        goal.target_pose.orientation.w = 1.0

        self.qr_ctrl_client.send_goal(goal)
        self.qr_ctrl_client.wait_for_result()
        if (self.qr_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_reach(self, arg):
        'SE3 Ctrl w/wo wholebody motion'
        if (len(arg) > 0):
            if (int(arg) == 0):
                print ("Only arm Control")
                goal = kimm_action_manager.msg.SE3Goal
                goal.duration = 5.0
                goal.target_pose = Pose()
                goal.target_pose.position.x = -0.05
                goal.target_pose.position.y = 0.05
                goal.target_pose.position.z = -0.1

                goal.target_pose.orientation.x = 0
                goal.target_pose.orientation.y = 0
                goal.target_pose.orientation.z = 0.
                goal.target_pose.orientation.w = 1

                goal.relative = True
                goal.wholebody = False

                self.se3_ctrl_client.send_goal(goal)
                self.se3_ctrl_client.wait_for_result()
                if (self.se3_ctrl_client.get_result()):
                    print ("action succeed")
                else:
                    print ("action failed")
        else:
            print ("Wholebody Control")
            goal = kimm_action_manager.msg.SE3Goal
            goal.duration = 10.0
            goal.target_pose = Pose()
            goal.target_pose.position.x = 3.2
            goal.target_pose.position.y = -0.185
            goal.target_pose.position.z = 0.5

            goal.target_pose.orientation.x = 0.
            goal.target_pose.orientation.y = 0
            goal.target_pose.orientation.z = 0
            goal.target_pose.orientation.w = 1.0

            goal.relative = False
            goal.wholebody = True

            self.se3_ctrl_client.send_goal(goal)
            self.se3_ctrl_client.wait_for_result()
            if (self.se3_ctrl_client.get_result()):
                print ("action succeed")
            else:
                print ("action failed")

    def do_move(self, arg):
        'Move'
        goal = kimm_action_manager.msg.MoveGoal

        goal.target_pose = PoseStamped()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 0.885
        goal.target_pose.pose.position.y = 0.105
        goal.target_pose.pose.position.z = 0.0

        goal.target_pose.pose.orientation.x = 0 
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0.2
        goal.target_pose.pose.orientation.w = 0.980

        self.move_ctrl_client.send_goal(goal)
        self.move_ctrl_client.wait_for_result()

        if (self.move_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
    

    def do_quit(self, arg):
        os.killpg(os.getpgid(self.savemap.pid), signal.SIGTERM)

        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()

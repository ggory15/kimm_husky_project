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
        self.joint_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/joint_posture_control', kimm_action_manager.msg.JointPostureAction)
        self.joint_ctrl_client.wait_for_server()
        self.se3_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/se3_control', kimm_action_manager.msg.SE3Action)
        self.se3_ctrl_client.wait_for_server()
        self.se3_array_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/se3_array_control', kimm_action_manager.msg.SE3ArrayAction)
        self.se3_array_ctrl_client.wait_for_server()
        self.gripper_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/gripper_control', kimm_action_manager.msg.GripperAction)
        self.gripper_ctrl_client.wait_for_server()
        self.move_ctrl_client = actionlib.SimpleActionClient('/ns0/kimm_action_manager/move_control', kimm_action_manager.msg.MoveAction)
        self.move_ctrl_client.wait_for_server()

        self.gravity_pub = rospy.Publisher('/ns0/kimm_action_manager/gravity_ctrl', Bool, queue_size=1)
        self.gravity = True
    
    def do_home(self, arg):
        'Go to the home position using joint posture ctrl'
        goal = kimm_action_manager.msg.JointPostureGoal
        goal.duration = 2.0
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

    def do_reach(self, arg):
        'SE3 Ctrl w/wo wholebody motion'
        if (len(arg) > 0):
            if (int(arg) == 0):
                print ("Only arm Control")
                goal = kimm_action_manager.msg.SE3Goal
                goal.duration = 2.0
                goal.target_pose = Pose()
                goal.target_pose.position.x = -0.05
                goal.target_pose.position.y = 0.05
                goal.target_pose.position.z = -0.2

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
            goal.duration = 2.0
            goal.target_pose = Pose()
            goal.target_pose.position.x = -0.2
            goal.target_pose.position.y = 0.1
            goal.target_pose.position.z = 0.0

            goal.target_pose.orientation.x = 0.258819
            goal.target_pose.orientation.y = 0
            goal.target_pose.orientation.z = 0.
            goal.target_pose.orientation.w = 0.9659258

            goal.relative = True
            goal.wholebody = True

            self.se3_ctrl_client.send_goal(goal)
            self.se3_ctrl_client.wait_for_result()
            if (self.se3_ctrl_client.get_result()):
                print ("action succeed")
            else:
                print ("action failed")

    def do_continous(self, arg):
        goal = kimm_action_manager.msg.SE3ArrayGoal
        goal.relative = []
        goal.wholebody = []

        goal.relative.append(True)
        goal.wholebody.append(False)
        goal.target_poses = PoseArray()
        pose_se = Pose()   
        pose_se.position.x = 0.0
        pose_se.position.y = 0.0
        pose_se.position.z = -0.2
        pose_se.orientation.x =  0.0
        pose_se.orientation.y = 0.0
        pose_se.orientation.z = 0.0
        pose_se.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se)

        goal.relative.append(True)
        goal.wholebody.append(True)
        pose_se2 = Pose()
        pose_se2.position.x = 0.2
        pose_se2.position.y = -0.2
        pose_se2.position.z = 0.0
        pose_se2.orientation.x =  0.0
        pose_se2.orientation.y = 0.0
        pose_se2.orientation.z = 0.0
        pose_se2.orientation.w = 1.0
        goal.target_poses.poses.append(pose_se2)
        
        goal.durations = Float32MultiArray()
        goal.durations.data.append(2.0)
        goal.durations.data.append(3.0)
        self.se3_array_ctrl_client.send_goal(goal)
        
        self.se3_array_ctrl_client.wait_for_result()
        if (self.se3_array_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_grasp(self, arg):
        'Grasping'
        if (len(arg) > 0):
            if (int(arg) == 0):
                print ("Close")
                goal = kimm_action_manager.msg.GripperGoal
                goal.grasp = True
        else:
            print ("Open")
            goal = kimm_action_manager.msg.GripperGoal
            goal.grasp = False

        self.gripper_ctrl_client.send_goal(goal)
        self.gripper_ctrl_client.wait_for_result()
        if (self.gripper_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

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

        self.move_ctrl_client.send_goal(goal)
        self.move_ctrl_client.wait_for_result()

        if (self.move_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_quit(self, arg):
        return True

if __name__ == '__main__':
    ControlSuiteShell().cmdloop()
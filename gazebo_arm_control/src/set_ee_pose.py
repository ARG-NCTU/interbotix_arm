#!/usr/bin/env python
from gettext import find
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import sys
import rospy
from std_msgs.msg import Bool, Float64, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
#If you want to know how to use InterbotixManipulatorXS check interbotix_commmon_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py file for more information



class gazebo_interbotix_sdk_bridge:
    def __init__(self, robot_name):
        #pose to joint_state using interbotix_sdk
        self.sub_end_effector_pose = rospy.Subscriber("/end_effector_pose", Pose, self.pose_callback, queue_size=1)
        self.pub_joint_state_from_catersian  = rospy.Publisher("/joint_states_from_api", JointState, queue_size=1)
        self.bot = InterbotixManipulatorXS(robot_model = robot_name, group_name = "arm", gripper_name = "gripper", init_node = False, moving_time = 0.5, accel_time=0.3) 
        if (self.bot.arm.group_info.num_joints < 5):
            print('This demo requires the robot to have at least 5 joints!')
            sys.exit()
        
        #joint_state to gazebo arm
        self.sub_jointstate = rospy.Subscriber("/joint_states_from_api", JointState, self.jointstate_callback, queue_size=1)
        #publish message to gazebo_robot (for position_control)
        self.pub_right_finger_gripper = rospy.Publisher("/" + robot_name + "/right_finger_controller/command",Float64,queue_size=1)
        self.pub_left_finger_gripper = rospy.Publisher("/" + robot_name + "/left_finger_controller/command",Float64,queue_size=1)
        self.pub_waist = rospy.Publisher("/" + robot_name + "/waist_controller/command",Float64,queue_size=1)
        self.pub_elbow = rospy.Publisher("/" + robot_name + "/elbow_controller/command",Float64,queue_size=1)
        self.pub_shoulder = rospy.Publisher("/" + robot_name + "/shoulder_controller/command",Float64,queue_size=1)
        self.pub_wrist_angle = rospy.Publisher("/" + robot_name + "/wrist_angle_controller/command",Float64,queue_size=1)
        self.pub_wrist_rotate = rospy.Publisher("/" + robot_name + "/wrist_rotate_controller/command",Float64,queue_size=1)
        
        start = rospy.Service("/initial", Trigger, self.initial)

    def pose_callback(self, pose_data):
        joint_state = JointState()
        joint_angle_list, find_ans = self.bot.arm.set_ee_pose_components(x=pose_data.position.x,y=pose_data.position.y,z=pose_data.position.z)
        if find_ans:
            print(joint_angle_list)
            joint_state.header.stamp = joint_state.header.stamp = rospy.Time.now()
            joint_state.position = joint_angle_list
            self.pub_joint_state_from_catersian.publish(joint_state)
        else:
            print("No valid solution")
        print("------------------------------------------------------------------")

    def pub_arm(self, joint_value):
        self.pub_waist.publish(joint_value[0])
        self.pub_shoulder.publish(joint_value[1])
        self.pub_elbow.publish(joint_value[2])
        self.pub_wrist_angle.publish(joint_value[3])
        self.pub_wrist_rotate.publish(joint_value[4])

    def jointstate_callback(self, joint_data):
        self.pub_arm(joint_data.position)
    

    def initial(self, req):
        res = TriggerResponse()
        try:
            joint_value =[0.0, 0.8, -1.0, 0.0, 0.0] #for upside down case
            # joint_value =[0.0, -1.5, 2.0, 0.0, 0.0]
            self.pub_arm(joint_value)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res



   

if __name__  == "__main__":
    rospy.init_node("gazebo_sdk_bridge")
    #robot_name = "wx250"
    robot_name = rospy.get_param("~robot_name")
    gazebo_interbotix_sdk_bridge(robot_name)
    rospy.spin()
    

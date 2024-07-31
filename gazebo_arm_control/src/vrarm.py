#!/usr/bin/env python2
################################
# The interbotix_robot_arm in gazebo is different from the real one,this code is for unity to control gazebo wx250 robot arm.
# joint_value from vr is ['waist (shoulder in vr)', 'shoulder (elbow in vr)', 'elbow (forearm in vr)', 'wrist_angle (wrist in vr)', 'wrist_rotate (gripper in vr)']
################################
import rospy
import math
import time
from std_msgs.msg import Bool, Float64, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from interbotix_xs_modules.arm import InterbotixManipulatorXS

class gazebo_arm_control():
    def __init__(self, robot_name="wx200", use_sim=True):
        #subscribe message from unity
        self.sub_primary = rospy.Subscriber("/vr/right/primarybutton", Bool, self.primarybutton_callback,queue_size=1)	#gripper open
        #self.sub_second = rospy.Subscriber("/vr/right/secondarybutton", Bool, self.secondarybutton_callback,queue_size=1) #gripper close
        self.sub_jointstate = rospy.Subscriber("/joint_states_test", JointState, self.jointstate_callback, queue_size=1)
        self.use_sim = use_sim
        if self.use_sim:  
            #publish message to gazebo_robot (for position_control)
            self.pub_right_finger_gripper = rospy.Publisher("/" + robot_name + "/right_finger_controller/command",Float64,queue_size=1)
            self.pub_left_finger_gripper = rospy.Publisher("/" + robot_name + "/left_finger_controller/command",Float64,queue_size=1)
            self.pub_waist = rospy.Publisher("/" + robot_name + "/waist_controller/command",Float64,queue_size=1)
            self.pub_elbow = rospy.Publisher("/" + robot_name + "/elbow_controller/command",Float64,queue_size=1)
            self.pub_shoulder = rospy.Publisher("/" + robot_name + "/shoulder_controller/command",Float64,queue_size=1)
            self.pub_wrist_angle = rospy.Publisher("/" + robot_name + "/wrist_angle_controller/command",Float64,queue_size=1)
            self.pub_wrist_rotate = rospy.Publisher("/" + robot_name + "/wrist_rotate_controller/command",Float64,queue_size=1)

        #rosservice for initial position and special pose
        start = rospy.Service("/initial", Trigger, self.initial)
        calibration = rospy.Service("/flypose", Trigger, self.flypose)
        gripper_open = rospy.Service("/gripper_open", Trigger, self.gripper_open)
        gripper_close = rospy.Service("/gripper_close", Trigger, self.gripper_close)
        push = rospy.Service("/push", Trigger, self.push)
        grasp_pose = rospy.Service("/grasp_pose", Trigger, self.grasp_pose)

        self.bot = InterbotixManipulatorXS(robot_model = robot_name, group_name = "arm", gripper_name = "gripper", init_node = False, moving_time = 0.5, accel_time=0.3)

    def initial(self, req):
        res = TriggerResponse()
        try:
            #joint_value =[0.0, 0.8, -1.0, 0.0, 0.0] #for upside down case
            joint_value = [0.0009009980741545576, -1.3184024200773958, 1.617598039350118, -0.30828725668373025, -0.00013571852838190068]
            if self.use_sim:
                self.pub_arm(joint_value)
            else:
                moving_time, accel_time = self.calculate_times(joint_value)
                self.bot.arm.set_joint_positions(joint_value, moving_time=moving_time, accel_time=accel_time)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res
    def flypose(self, req):
        res = TriggerResponse()
        try:
            joint_value = [0.0004, -0.609, 0.808, -1.154, 0]
            if self.use_sim:
                self.pub_arm(joint_value)
            else:
                moving_time, accel_time = self.calculate_times(joint_value)
                self.bot.arm.set_joint_positions(joint_value, moving_time=moving_time, accel_time=accel_time)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res

    def grasp_pose(self, req):
        res = TriggerResponse()
        try:
            joint_value = [0.0, 0.0, 0.0, 0.0, 0.0]
            if self.use_sim:
                self.pub_arm(joint_value)
            else:
                moving_time, accel_time = self.calculate_times(joint_value)
                self.bot.arm.set_joint_positions(joint_value, moving_time=moving_time, accel_time=accel_time)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res
        
    def gripper_open(self, req):
        res = TriggerResponse()
        try:
            if self.use_sim:
                self.pub_finger("open")
            else:
                self.bot.gripper.open()
            
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res

    def gripper_close(self, req):
        res = TriggerResponse()
        try:
            if self.use_sim:
                self.pub_finger("close")
            else:
                self.bot.gripper.close()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res
    
    def push(self, req):
        res = TriggerResponse()
        try:
            joint_value_list = [[0.0, -0.4, 0.25, 0.0, 0.0], [0.0, -0.8, 0.5, 0.0, 0.0], [0.0, -0.4, 0.25, 0.0, 0.0], [0.0, 0.8, -1.0, 0.0, 0.0]]
            for joint_value in joint_value_list:
                if self.use_sim:
                    self.pub_arm(joint_value)
                else:
                    moving_time, accel_time = self.calculate_times(joint_value)
                    self.bot.arm.set_joint_positions(joint_value, moving_time=moving_time, accel_time=accel_time)
                time.sleep(0.5)
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        return res


    def pub_arm(self, joint_value):
        self.pub_waist.publish(joint_value[0])
        self.pub_shoulder.publish(joint_value[1])
        self.pub_elbow.publish(joint_value[2])
        self.pub_wrist_angle.publish(joint_value[3])
        self.pub_wrist_rotate.publish(-joint_value[4]) # multiply -1 for reverse arm

    def pub_finger(self, finger_state):
        if finger_state == "open":
            print("open")
            self.pub_right_finger_gripper.publish(-1.0)
            self.pub_left_finger_gripper.publish(1.0)
        elif finger_state == "close":
            print("close")
            self.pub_right_finger_gripper.publish(0.0)
            self.pub_left_finger_gripper.publish(0.0)

    def primarybutton_callback(self, msg):
        if msg.data == True:
            if self.use_sim:
                self.pub_finger("open")
            else:
                self.bot.gripper.open()
        else:
            if self.use_sim:
                self.pub_finger("close")
            else:
                self.bot.gripper.close()

    # def secondarybutton_callback(self, msg):
    #     if msg.data == True:
    #         if self.use_sim:
    #             self.pub_finger("close")
    #         else:
    #             self.bot.gripper.close()
    #     else:
    #         pass
    def jointstate_callback(self, joint_data):
        if self.use_sim:
            self.pub_arm(joint_data.position)
        else:
            moving_time, accel_time = self.calculate_times(joint_data.position)
            self.bot.arm.set_joint_positions(joint_data.position, moving_time=moving_time, accel_time=accel_time)
        #print(self.joint_value)

    def calculate_times(self, joint_value):
        # Get current joint positions
        current_positions = self.bot.arm.get_joint_commands()
        
        # Calculate the maximum distance to be moved
        max_distance = max(abs(curr - target) for curr, target in zip(current_positions, joint_value))
        
        # Set base times
        base_moving_time = 0.3  # seconds for small movements
        base_accel_time = 0.1   # seconds for small movements
        
        # Scale times based on the maximum distance
        moving_time = base_moving_time + max_distance * 1.5  # Adjust scaling factor as needed
        accel_time = base_accel_time + max_distance * 0.5  # Adjust scaling factor as needed
        
        # Ensure accel_time is not more than half of moving_time
        accel_time = min(accel_time, moving_time / 2)
        
        # Check if the velocities are within limits
        velocity_limits = self.bot.arm.group_info.joint_velocity_limits
        for curr, target, limit in zip(current_positions, joint_value, velocity_limits):
            velocity = abs(target - curr) / moving_time
            if velocity > limit:
                # Adjust moving_time and accel_time to ensure velocity limits are not exceeded
                moving_time = abs(target - curr) / limit
                accel_time = min(accel_time, moving_time / 2)
        
        return moving_time, accel_time


        

if __name__ == "__main__":
    rospy.init_node("vr_gazebo_arm_control")
    robot_name = rospy.get_param("~robot_name")
    sim = rospy.get_param("~use_sim")
    initial = rospy.get_param("~initial")
    gazebo_arm_control(robot_name, sim)
    if initial:
        rospy.wait_for_service("/initial")
        try:
            initial = rospy.ServiceProxy("/initial", Trigger)
            initial()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    rospy.spin()

#!/usr/bin/env python
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import sys
import rospy
from std_msgs.msg import Bool, Float64, Empty
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
import tf
import time 
#If you want to know how to use InterbotixManipulatorXS check interbotix_commmon_toolbox/interbotix_xs_modules/src/interbotix_xs_modules/arm.py file for more information


class gazebo_interbotix_sdk_bridge:
    def __init__(self, robot_name='wx200', use_sim=True):
        #pose to joint_state using interbotix_sdk
        self.sub_end_effector_pose = rospy.Subscriber("/control_end_effector_pose", Pose, self.pose_callback, queue_size=1)
        self.pub_joint_state_from_catersian  = rospy.Publisher("/joint_states_from_api", JointState, queue_size=1)
        self.pub_ee_pose = rospy.Publisher("/end_effector_pose", PoseStamped, queue_size=1)
        self.pub_global_ee_pose = rospy.Publisher("/global_end_effector_pose", PoseStamped, queue_size=1)
        self.bot = InterbotixManipulatorXS(robot_model = robot_name, group_name = "arm", gripper_name = "gripper", init_node = False, moving_time = 1, accel_time=0.3) 
        if (self.bot.arm.group_info.num_joints < 5):
            print('This demo requires the robot to have at least 5 joints!')
            sys.exit()
        self.use_sim = use_sim

        #self.sub_jointstate = rospy.Subscriber("/joint_states_from_api", JointState, self.jointstate_callback, queue_size=1)
        if self.use_sim:
            #publish message to gazebo_robot (for position_control)
            self.pub_right_finger_gripper = rospy.Publisher("/" + robot_name + "/right_finger_controller/command",Float64,queue_size=1)
            self.pub_left_finger_gripper = rospy.Publisher("/" + robot_name + "/left_finger_controller/command",Float64,queue_size=1)
            self.pub_waist = rospy.Publisher("/" + robot_name + "/waist_controller/command",Float64,queue_size=1)
            self.pub_elbow = rospy.Publisher("/" + robot_name + "/elbow_controller/command",Float64,queue_size=1)
            self.pub_shoulder = rospy.Publisher("/" + robot_name + "/shoulder_controller/command",Float64,queue_size=1)
            self.pub_wrist_angle = rospy.Publisher("/" + robot_name + "/wrist_angle_controller/command",Float64,queue_size=1)
            self.pub_wrist_rotate = rospy.Publisher("/" + robot_name + "/wrist_rotate_controller/command",Float64,queue_size=1)
        
        self.draw_ellipse_service = rospy.Service('draw_ellipse', Trigger, self.draw_ellipse_callback)
        self.timer = rospy.Timer(rospy.Duration(1/5.0), self.publish_ee_pose_callback)

        self.listener = tf.TransformListener()

    def pose_callback(self, pose_data):
        joint_state = JointState()
        joint_angle_list, find_ans = self.bot.arm.set_ee_pose_components(x=pose_data.position.x,y=pose_data.position.y,z=pose_data.position.z, execute=True, moving_time=1.0, accel_time=3, blocking=True)
        if find_ans:
            print(joint_angle_list)
            joint_state.header.stamp = joint_state.header.stamp = rospy.Time.now()
            joint_state.position = joint_angle_list
            #self.pub_joint_state_from_catersian.publish(joint_state)
            if self.use_sim:
                self.pub_arm(joint_state.position)
        else:
            print("No valid solution")
        print("------------------------------------------------------------------")

    def pub_arm(self, joint_value):
        self.pub_waist.publish(joint_value[0])
        self.pub_shoulder.publish(joint_value[1])
        self.pub_elbow.publish(joint_value[2])
        self.pub_wrist_angle.publish(joint_value[3])
        self.pub_wrist_rotate.publish(joint_value[4])

    # def jointstate_callback(self, joint_data):
    #     self.pub_arm(joint_data.position)
    

    def control_end_effector(self, key):
        joint_state = JointState()
        dx, dy, dz = 0, 0, 0
        displacement = 0.01  # 0.01 meters displacement

        if key == 'w':
            dx = displacement
        elif key == 's':
            dx = -displacement
        elif key == 'a':
            dy = -displacement
        elif key == 'd':
            dy = displacement
        elif key == 'z':
            dz = -displacement
        elif key == 'x':
            dz = displacement

        joint_angle_list, find_ans = self.bot.arm.set_relative_ee_position_wrt_to_base_frame(dx=dx, dy=dy, dz=dz, execute=True, moving_time=0.1, accel_time=0.1, blocking=True)
        if find_ans:
            print(joint_angle_list)
            joint_state.header.stamp = joint_state.header.stamp = rospy.Time.now()
            joint_state.position = joint_angle_list
            #self.pub_joint_state_from_catersian.publish(joint_state)
            if self.use_sim:
                self.pub_arm(joint_state.position)
        else:
            print("No valid solution")
        print("------------------------------------------------------------------")

    # use static ellipse trajectory for robot arm
    def calculate_ellipse_trajectory(self, num_points=100):
        t1 = np.linspace(4.5, 40+4.5, num_points)
        t2 = np.linspace(40+4.5, 4.5, num_points)
        t = np.concatenate((t1, t2))
        x = 0.41 + 0.077 * np.sin(0.35 * t)
        y = 0.08 * np.cos(0.35 * t) 
        z = 0.3 + 0.05 * np.sin(0.35 * t)

        return list(zip(x, y, z))
    
    def draw_ellipse_callback(self, request):
        # Calculate the ellipse trajectory
        rospy.ServiceProxy('grasp_pose', Trigger)()
        trajectory = self.calculate_ellipse_trajectory()
        rospy.loginfo("Drawing ellipse trajectory...")
        first_point = True
        for x, y, z in trajectory:
            if first_point:
                joint_angle_list, find_ans = self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, moving_time=1, accel_time=0.3)
                first_point = False
            rospy.loginfo("Moving to x={}, y={}, z={}".format(x, y, z))
            joint_angle_list, find_ans = self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, moving_time=0.05, accel_time=0.01)
            if find_ans:
                print(joint_angle_list)
                joint_state = JointState()
                joint_state.header.stamp = joint_state.header.stamp = rospy.Time.now()
                joint_state.position = joint_angle_list
                if self.use_sim:
                    self.pub_arm(joint_state.position)
        return TriggerResponse()  # Indicate successful execution
    

    def publish_ee_pose_callback(self, event):
        pose_matrix = self.bot.arm.get_ee_pose()  # get the pose as a 4x4 transformation matrix
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"

        # Extract the position from the transformation matrix
        pose_msg.pose.position.x = pose_matrix[0, 3]
        pose_msg.pose.position.y = pose_matrix[1, 3]
        pose_msg.pose.position.z = pose_matrix[2, 3]

        # Assuming you need to convert the rotation matrix to a quaternion
        quaternion = tf.transformations.quaternion_from_matrix(pose_matrix)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        self.pub_ee_pose.publish(pose_msg)
        if self.use_sim:
            try:
                (trans, rot) = self.listener.lookupTransform('map', 'wx200/ee_gripper_link', rospy.Time(0))
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                pose_stamped.header.stamp = rospy.Time.now()  # or use rospy.Time(0) if you want the time of the transform
                pose_stamped.pose.position.x = trans[0]
                pose_stamped.pose.position.y = trans[1]
                pose_stamped.pose.position.z = trans[2]
                pose_stamped.pose.orientation.x = rot[0]
                pose_stamped.pose.orientation.y = rot[1]
                pose_stamped.pose.orientation.z = rot[2]
                pose_stamped.pose.orientation.w = rot[3]
                self.pub_global_ee_pose.publish(pose_stamped)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.loginfo("Error in getting transform: %s" % e)
        
if __name__  == "__main__":
    rospy.init_node("gazebo_sdk_bridge")
    #robot_name = "wx250"
    robot_name = rospy.get_param("~robot_name")
    use_sim = rospy.get_param("~use_sim")
    gazebo_control = gazebo_interbotix_sdk_bridge(robot_name, use_sim)
    key_board_control = rospy.get_param("~keyboard_control")
    while not rospy.is_shutdown():
        if key_board_control:
            key = raw_input("please input key: ").strip()
            if key == 'q':
                break
            else:
                gazebo_control.control_end_effector(key)
        else:
            pass
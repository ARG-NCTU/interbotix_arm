#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import JointState


class fix_endeffector_rotate():
	def __init__(self):
		self.sub_jointstate = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback, queue_size=1)
		self.pub_jointstate_fix = rospy.Publisher("/joint_states_fix", JointState, queue_size=1)
		self.pub_jointstate = JointState()
	
	def jointstate_callback(self, joint_data):
		self.pub_jointstate = JointState()
		self.pub_jointstate.name = joint_data.name
		self.pub_jointstate.position = list(joint_data.position)  # Make a copy of the position list
		self.pub_jointstate.velocity = joint_data.velocity
		self.pub_jointstate.effort = joint_data.effort
		
		# Modify the 'wrist_rotate' joint position
		for i, name in enumerate(joint_data.name):
			if name == "wrist_rotate":
				self.pub_jointstate.position[i] = -joint_data.position[i]  # Invert the position
				break
		self.pub_jointstate_fix.publish(self.pub_jointstate)
if __name__ == '__main__':
	rospy.init_node('fix_endeffector_rotate', anonymous=True)
	fix_endeffector_rotate()
	rospy.spin()
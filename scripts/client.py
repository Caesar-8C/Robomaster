#!/usr/bin/env python3
import rospy
from robomaster_controller.msg import Params
from robomaster_controller.msg import Info



class keyCap:

	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		listener = rospy.Subscriber('/moo', Params, self.callback, queue_size=1)
		self.publisher = rospy.Publisher('/pdError', Info, queue_size=1)

		while not rospy.is_shutdown():
			pass

	def callback(self, data):
		self.p = data.pdcontrol.p
		self.d = data.pdcontrol.d
		self.target = [data.target.z, data.target.x, data.target.theta]
		self.speed = [data.speed.linear, data.speed.angular]
		self.publish()


	def publish(self):
		msg = Info()
		msg.pdcontrol.p = self.p
		msg.pdcontrol.d = self.d
		msg.rotation = 8
		msg.pose.z = self.target[0]
		msg.pose.x = self.target[1]
		msg.pose.theta = self.target[2]
		msg.sticks.lv = self.speed[0]
		msg.sticks.lh = self.speed[1]
		msg.sticks.rh = 9
		self.publisher.publish(msg)

if __name__ == '__main__':
	keyCap()
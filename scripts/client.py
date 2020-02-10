#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from robomaster_controller.msg import Params
from robomaster_controller.msg import Info
import websocket


def on_message(ws, message):
	print('Message from server:', message)


class keyCap:

	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		listener =  rospy.Subscriber('/tracker/pose', PoseStamped, self.callback, queue_size=1)
		self.ws = websocket.WebSocketApp('ws://192.168.1.143:8181', on_message=on_message)

		self.ws.run_forever()
		while not rospy.is_shutdown():
			pass
		self.ws.close()



	def callback(self, data):
		msg = '0.0 0.0 0.0 0.0'
		self.ws.send(msg)


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
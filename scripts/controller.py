#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from robomaster_controller.msg import Params
from robomaster_controller.msg import Info

import websocket
import time
import numpy as np



class RobotControlClient:

	def __init__(self):
		rospy.init_node('listener', anonymous=True)

		self.target = [-1., 0., 0.]
		self.speed = [0.5, 0.1]

		self.pGain = 2.
		self.dGain = 0.05

		self.lastError = 0
		self.lastTime = 0

		self.pitch = 0
		self.baseYaw = 0
		self.gimbalYaw = 0

		self.p = 0
		self.d = 0

		self.publisher = rospy.Publisher('/robomaster/info', PoseStamped, queue_size=1)
		listener =  rospy.Subscriber('/tracker/pose', PoseStamped, self.robotCallback, queue_size=1)
		listener2 = rospy.Subscriber('/cmd_vel', Params, self.commandCallback, queue_size=1)
		listener3 = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.targetCallback, queue_size=1)

		self.ws = websocket.WebSocketApp('ws://192.168.1.143:8181', on_message=self.on_message)
		self.ws.run_forever()
		while not rospy.is_shutdown():
			pass
		self.ws.close()


	def on_message(self, message):
		msgList = message.split()
		if len(msgList) == 3:
			self.pitch = np.radians(float(msgList[0]))
			self.baseYaw = -np.radians(float(msgList[1]))
			self.gimbalYaw = -np.radians(float(msgList[2]))
		else:
			rospy.loginfo(message)


	def commandCallback(self, data):
		self.pGain = data.pdcontrol.p
		self.dGain = data.pdcontrol.d
		# self.target = [data.target.z, data.target.x, data.target.theta]
		self.speed = [data.speed.linear, data.speed.angular]


	def targetCallback(self, data):
		z = data.pose.position.x
		x = data.pose.position.y

		q = data.pose.orientation
		q.y = q.z
		q.z = 0.0

		theta = self.quat2Rotation(q)

		self.target = [z, x, theta]
		rospy.loginfo('New target: ' + str(self.target))


	def publish(self, movementVector, rotationCommand):
		msg = Info()
		msg.pdcontrol.p = self.p
		msg.pdcontrol.d = self.d
		msg.pose.z = self.z
		msg.pose.x = self.x
		msg.pose.theta = self.gimbalAngle
		msg.sticks.lv = movementVector[0]
		msg.sticks.lh = movementVector[1]
		msg.sticks.rh = rotationCommand
		msg = PoseStamped()
		msg.header.frame_id = "map"
		msg.pose.position.x = self.z
		msg.pose.position.y = self.x
		msg.pose.orientation.z = np.sin(self.gimbalAngle/2.)
		msg.pose.orientation.w = np.cos(self.gimbalAngle/2.)
		self.publisher.publish(msg)


	def pdControl(self, z, x):
		error = np.linalg.norm(np.array([self.target[0]-z, self.target[1]-x]))
		self.p = self.pGain * error

		if self.lastTime == 0:
			self.lastTime = time.time()
			return self.p

		timeNow = time.time()
		deltaT = timeNow - self.lastTime
		self.lastTime = timeNow

		errorDot = (error - self.lastError) / deltaT
		self.d = self.dGain * errorDot
		self.lastError = error

		return self.p + self.d


	def quat2Rotation(self, q):
		angle = -np.arcsin(2*(q.x*q.z - q.w*q.y))
		robotFacingBackwards = 1-2*(q.y**2 + q.z**2) < 0
		if robotFacingBackwards:
			if angle < 0:
				angle = -(np.pi + angle)
			else:
				angle = np.pi - angle
		return angle


	def computeMovementVector(self):
		robotForwardVector = np.array([np.cos(self.baseAngle), np.sin(self.baseAngle)])
		robot2TargetVector = np.array([self.target[0] - self.z, self.target[1] - self.x])

		robot2TargetAngle = np.arccos(np.dot(robotForwardVector, robot2TargetVector)/(np.linalg.norm(robotForwardVector)*np.linalg.norm(robot2TargetVector)))
		robot2TargetAngle_sign = np.sign(np.cross(robotForwardVector, robot2TargetVector))
		robot2TargetAngle *= robot2TargetAngle_sign

		movementVector = np.array([np.cos(robot2TargetAngle), -np.sin(robot2TargetAngle)]) * self.speed[0] * self.pdControl(self.z, self.x)

		maxVal = np.max(np.abs(movementVector))
		if maxVal > 1:
			movementVector /= maxVal

		return movementVector


	def computeRotationCommand(self):
		if np.abs(self.target[2] - self.gimbalAngle < np.pi):
			rotationCommand = -(self.target[2] - self.gimbalAngle) * self.speed[1]
		else:
			rotationCommand = (self.target[2] - self.gimbalAngle) * self.speed[1]
		return rotationCommand


	def checkIfGoalReached(self):
		positionGoalReached = np.abs(self.target[0] - self.z)<0.1 and np.abs(self.target[1] - self.x)<0.1
		rotationGoalReached = np.abs(self.gimbalAngle - self.target[2])<0.035
		if positionGoalReached and rotationGoalReached:
			rospy.loginfo_throttle(3, 'Goal Reached: '+str(self.target))
		return positionGoalReached, rotationGoalReached


	def robotCallback(self, data):
		self.x = data.pose.position.x
		self.y = data.pose.position.y
		self.z = data.pose.position.z
		lostSignalCheck = np.abs(self.x) + np.abs(self.y) + np.abs(self.z) < 0.01
		if lostSignalCheck:
			msg = '0.0 0.0 0.0 0.0'
			rospy.loginfo_throttle(0.5, 'Lost Tracking Signal')
			try:
				self.ws.send(msg)
			except:
				rospy.loginfo_throttle(0.5, 'No WS Connection')
			return
			
		self.gimbalAngle = self.quat2Rotation(data.pose.orientation)
		self.baseAngle = self.gimbalAngle - self.gimbalYaw

		movementVector = self.computeMovementVector()
		rotationCommand = self.computeRotationCommand()

		positionGoalReached, rotationGoalReached = self.checkIfGoalReached()
		if positionGoalReached:
			movementVector = np.array([0.0, 0.0])
		if rotationGoalReached:
			rotationCommand = 0.0

		msg = str(movementVector[0]) + ' ' + str(movementVector[1]) + ' ' + str(0.0) + ' ' + str(rotationCommand)
		try:
			self.ws.send(msg)
		except:
			rospy.loginfo_throttle(0.5, 'No WS Connection')
		# rospy.loginfo(str(self.gimbalYaw))

		self.publish(movementVector, rotationCommand)



if __name__ == '__main__':
	RobotControlClient()
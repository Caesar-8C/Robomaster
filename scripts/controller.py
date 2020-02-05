#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

import websocket
import time
import numpy as np


class RobotControlClient:

	def __init__(self):
		rospy.init_node('listener', anonymous=True)
		listener = rospy.Subscriber('/tracker/pose', PoseStamped, self.robotCallback, queue_size=1)
		listener2 = rospy.Subscriber('/cmd_vel', Twist, self.commandCallback, queue_size=1)
		self.publisher = rospy.Publisher('/pdError', Twist, queue_size=1)

		self.target = [-1., 0., 0.]
		self.speed = [0.5, 0.05]

		self.pGain = 2.
		self.dGain = 0.05

		self.lastError = 0
		self.lastTime = 0

		self.ws = websocket.WebSocketApp('ws://192.168.1.143:8181', on_message=self.on_message)
		while not rospy.is_shutdown():
			self.ws.run_forever()
		self.ws.close()


	def on_message(self, ws, message):
		print('Message from server:', message)


	def commandCallback(self, data):
		self.pGain = data.pdcontrol.p
		self.dGain = data.pdcontrol.d
		self.target = [data.target.z, data.target.x, data.target.theta]
		self.speed = [data.speed.linear, data.speed.angular]


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


	def computeRobotAngle(self, q):
		robotAngle = -np.arcsin(2*(q.x*q.z - q.w*q.y))
		robotFacingBackwards = 1-2*(q.y**2 + q.z**2) < 0
		if robotFacingBackwards:
			if robotAngle < 0:
				robotAngle = -(np.pi + robotAngle)
			else:
				robotAngle = np.pi - robotAngle
		return robotAngle


	def computeMovementVector(self):
		robotForwardVector = np.array([np.cos(self.robotAngle), np.sin(self.robotAngle)])
		robot2TargetVector = np.array([self.target[0] - self.z, self.target[1] - self.x])

		robot2TargetAngle = np.arccos(np.dot(robotForwardVector, robot2TargetVector)/(np.linalg.norm(robotForwardVector)*np.linalg.norm(robot2TargetVector)))
		robot2TargetAngle_sign = np.sign(np.cross(robotForwardVector, robot2TargetVector))
		robot2TargetAngle *= robot2TargetAngle_sign

		movementVector = np.array([-np.cos(robot2TargetAngle), np.sin(robot2TargetAngle)]) * self.speed[0] * self.pdControl(self.z, self.x)

		maxVal = np.max(np.abs(movementVector))
		if maxVal > 1:
			movementVector /= maxVal

		return movementVector


	def computeRotationCommand(self):
		if np.abs(self.target[2] - np.degrees(self.robotAngle) < 180):
			rotationCommand = -(self.target[2] - np.degrees(self.robotAngle)) * self.speed[1]
		else:
			rotationCommand = (self.target[2] - np.degrees(self.robotAngle)) * self.speed[1]
		return rotationCommand


	def checkIfGoalReached(self):
		positionGoalReached = np.abs(self.target[0] - self.z)<0.1 and np.abs(self.target[1] - self.x)<0.1
		rotationGoalReached = np.abs(np.degrees(self.robotAngle) - self.target[2])<2.0
		if positionGoalReached and rotationGoalReached:
			rospy.loginfo('\nGoal Reached\n'+str(self.target))
		return positionGoalReached, rotationGoalReached


	def publish(self, movementVector, rotationCommand):
		msg = Publish()
		msg.pdcontrol.p = self.p
		msg.pdcontrol.d = self.d
		msg.rotation = self.rotationCommand
		msg.pose.z = self.z
		msg.pose.x = self.x
		msg.pose.theta = self.robotAngle
		msg.sticks.lv = movementVector[0]
		msg.sticks.lh = movementVector[1]
		msg.sticks.rh = movementVector[2]
		self.publisher.publish(msg)


	def robotCallback(self, data):
		self.x = data.pose.position.x
		self.y = data.pose.position.y
		self.z = data.pose.position.z
		lostSignalCheck = np.abs(self.x) + np.abs(self.y) + np.abs(self.z) < 0.01
		if lostSignalCheck:
			msg = '0.0 0.0 0.0 0.0'
			rospy.loginfo('Lost Signal')
			self.ws.send(msg)
			return
			
		self.robotAngle = self.computeRobotAngle(data.pose.orientation)
		movementVector = self.computeMovementVector()
		rotationCommand = self.computeRotationCommand()

		positionGoalReached, rotationGoalReached = self.checkIfGoalReached()
		if positionGoalReached:
			movementVector = np.array([0.0, 0.0])
		if rotationGoalReached:
			rotationCommand = 0.0

		msg = str(movementVector[0]) + ' ' + str(movementVector[1]) + ' ' + str(0.0) + ' ' + str(rotationCommand)
		self.ws.send(msg)

		self.publish(movementVector, rotationCommand)



if __name__ == '__main__':
	RobotControlClient()
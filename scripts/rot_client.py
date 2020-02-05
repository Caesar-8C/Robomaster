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
		l = data.linear
		self.target = [l.x, l.y, l.z]
		a = data.angular
		self.speed = [a.x, 0.05]
		self.pGain = a.y
		self.dGain = a.z


	def pdControl(self, z, x):
		if self.lastTime == 0:
			return 1.

		error = np.linalg.norm(np.array([self.target[0]-z, self.target[1]-x]))
		p = self.pGain * error

		deltaT = time.time() - self.lastTime
		errorDot = (error - self.lastError) / deltaT
		d = self.dGain * errorDot

		self.lastError = error

		self.publishPD(error, (error - self.lastError), p, d, deltaT)
		return p + d


	def publishPD(self, error, errorDiff, p, d, deltaT):
		msg = Twist()
		msg.linear.x = error
		msg.linear.y = errorDiff
		msg.angular.x = p
		msg.angular.y = d
		msg.angular.z = deltaT
		self.publisher.publish(msg)


	def computeRobotAngle(self, q):
		robotAngle = -np.arcsin(2*(q.x*q.z - q.w*q.y))
		robotFacingBackwards = 1-2*(q.y**2 + q.z**2) < 0
		if robotFacingBackwards:
			if robotAngle < 0:
				robotAngle = -(np.pi + robotAngle)
			else:
				robotAngle = np.pi - robotAngle
		return robotAngle


	def robotCallback(self, data):
		lostSignalCheck = np.abs(data.pose.position.x) + np.abs(data.pose.position.y) + np.abs(data.pose.position.z) < 0.01
		if lostSignalCheck:
			msg = '0.0 0.0 0.0 0.0'
			rospy.loginfo('Lost Signal')
			self.ws.send(msg)
			return
			
		robotAngle = self.computeRobotAngle(data.pose.orientation)


		movementVector = np.array([1.0, 1.0])
		rotationCommand = 1.0


		positionGoalReached = np.abs(self.target[0] - data.pose.position.z)<0.0000001 and np.abs(self.target[1] - data.pose.position.x)<0.0000001
		rotationGoalReached = np.abs(np.degrees(robotAngle) - self.target[2])<2.0

		if positionGoalReached:
			movementVector = np.array([0.0, 0.0])
		if rotationGoalReached:
			rotationCommand = 0.0

		if positionGoalReached and rotationGoalReached:
			msg = '0.0 0.0 0.0 0.0'
			rospy.loginfo('\nGoal Reached\n'+str(self.target))
			self.ws.send(msg)
			return

		robotForwardVector = np.array([np.cos(robotAngle), np.sin(robotAngle), 0])
		robot2TargetVector = np.array([self.target[0] - data.pose.position.z, self.target[1] - data.pose.position.x, 0])

		robot2TargetAngle = np.arccos(np.dot(robotForwardVector, robot2TargetVector)/(np.linalg.norm(robotForwardVector)*np.linalg.norm(robot2TargetVector)))
		robot2TargetAngle_sign = np.cross(robotForwardVector, robot2TargetVector)
		robot2TargetAngle *= np.sign(robot2TargetAngle_sign[2])

		movementVector *= np.array([np.cos(robot2TargetAngle), np.sin(robot2TargetAngle)]) * self.speed[0] * self.pdControl(data.pose.position.z, data.pose.position.x)

		maxVal = np.max(np.abs(movementVector))
		if maxVal > 1:
			movementVector /= maxVal

		if positionGoalReached:
			if np.abs(self.target[2] - np.degrees(robotAngle) < 180):
				rotationCommand *= (self.target[2] - np.degrees(robotAngle)) * self.speed[1]
			else:
				rotationCommand *= -(self.target[2] - np.degrees(robotAngle)) * self.speed[1]
		else:
			rotationCommand = 0.0

		msg = str(movementVector[0]) + ' ' + str(-movementVector[1]) + ' ' + str(0.0) + ' ' + str(-rotationCommand)
		rospy.loginfo('\n'+msg+'\n'+str(data.pose.position.z)+' '+str(data.pose.position.x)+' \t'+str(np.degrees(robotAngle)))


		self.lastTime = time.time()

		self.ws.send(msg)



if __name__ == '__main__':
	RobotControlClient()
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
		listener =  rospy.Subscriber('/tracker/pose', PoseStamped, self.robotCallback, queue_size=1)
		listener2 = rospy.Subscriber('/cmd_vel', Params, self.commandCallback, queue_size=1)
		listener3 = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.targetCallback, queue_size=1)
		self.publisher = rospy.Publisher('/robomaster/info', PoseStamped, queue_size=1)

		self.target = [-1., 0., 0.]
		self.speed = [0.5, 0.1]

		self.pGain = 2.
		self.dGain = 0.05

		self.lastError = 0
		self.lastTime = 0

		self.pitch = 0
		self.globalYaw = 0
		self.yaw = 0

		self.ws = websocket.WebSocketApp('ws://192.168.1.143:8181', on_message=self.on_message)
		self.ws.run_forever()
		while not rospy.is_shutdown():
			pass
		self.ws.close()


	def on_message(self, message):
		msgList = message.split()
		if len(msgList) == 3:
			[self.pitch, self.globalYaw, self.yaw] = [float(x) for x in msgList]


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
		print(self.target)


	def publish(self, movementVector, rotationCommand):
		msg = Info()
		msg.pdcontrol.p = self.p
		msg.pdcontrol.d = self.d
		msg.pose.z = self.z
		msg.pose.x = self.x
		msg.pose.theta = self.robotAngle
		msg.sticks.lv = movementVector[0]
		msg.sticks.lh = movementVector[1]
		msg.sticks.rh = rotationCommand
		msg = PoseStamped()
		msg.header.frame_id = "map"
		msg.pose.position.x = self.z
		msg.pose.position.y = self.x
		msg.pose.orientation.z = np.sin(self.robotAngle/2.)
		msg.pose.orientation.w = np.cos(self.robotAngle/2.)
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

		movementVector = np.array([np.cos(robot2TargetAngle), -np.sin(robot2TargetAngle)]) * self.speed[0] * self.pdControl(self.z, self.x)

		maxVal = np.max(np.abs(movementVector))
		if maxVal > 1:
			movementVector /= maxVal

		return movementVector


	def computeRotationCommand(self):
		if np.abs(self.target[2] - self.robotAngle < np.pi):
			rotationCommand = -(self.target[2] - self.robotAngle) * self.speed[1]
		else:
			rotationCommand = (self.target[2] - self.robotAngle) * self.speed[1]
		return rotationCommand


	def checkIfGoalReached(self):
		positionGoalReached = np.abs(self.target[0] - self.z)<0.1 and np.abs(self.target[1] - self.x)<0.1
		rotationGoalReached = np.abs(self.robotAngle - self.target[2])<0.035
		if positionGoalReached and rotationGoalReached:
			rospy.loginfo('\nGoal Reached\n'+str(self.target))
		return positionGoalReached, rotationGoalReached


	def robotCallback(self, data):
		self.x = data.pose.position.x
		self.y = data.pose.position.y
		self.z = data.pose.position.z
		lostSignalCheck = np.abs(self.x) + np.abs(self.y) + np.abs(self.z) < 0.01
		if lostSignalCheck:
			msg = '0.0 0.0 0.0 0.0'
			# rospy.loginfo('Lost Signal')
			self.ws.send(msg)
			return
			
		self.robotAngle = self.quat2Rotation(data.pose.orientation)
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
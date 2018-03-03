#!/usr/bin/env python
import rospy
from std_msgs.msg import String
isObstacle = 0
motorLeft = 0
motorRight = 0
class ObstacleAvoidance:
	def getIsObstacle():
		global isObstacle
		return isObstacle
	def getMotorLeft():
		global motorLeft
		return motorLeft
	def getMotorRight():
		global motorRight
		return motorRight

def callback(data):
	parsedData = data.data.split(" ")
	print("isObstacle: " + parsedData[0] + "\nmotorLeft: " + parsedData[1] + "\nmotorRight: " + parsedData[2]);
	global isObstacle
	global motorLeft
	global motorRight
	isObstacle = int(parsedData[0])
	motorLeft = int(parsedData[1])
	motorRight = int(parsedData[2])
def listener():
	rospy.init_node("pythonsucks", anonymous=True)

	rospy.Subscriber("avoid", String, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
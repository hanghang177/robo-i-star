#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	parsedData = data.data.split(" ")
	print("isObstacle: " + parsedData[0] + "\nmotorLeft: " + parsedData[1] + "\nmotorRight: " + parsedData[2]);

def listener():
	rospy.init_node("pythonsucks", anonymous=True)

	rospy.Subscriber("avoid", String, callback)

	rospy.spin()

if __name__ == '__main__':
	listener()
from threading import Thread

class RosListener:
    def __init__(self):
        self.isObstacle = False
        self.motorLeft = 0
        self.motorRight = 0
        try:
            import rospy
        except:
            print("rospy not installed")
        else:
            from std_msgs.msg import String
            rospy.init_node("obstacle_info", anonymous=True)
            try:
                rospy.Subscriber("avoid", String, self.callback)
            except:
                print("ros failed")
            else:
                Thread(target=self.rosspin).start()

    def callback(self,data):
        parsedData = data.data.split(" ")
        self.isObstacle = int(parsedData[0])
        self.motorLeft = int(parsedData[1])
        self.motorRight = int(parsedData[2])
        print("isObstacle: " + parsedData[0] + "\nmotorLeft: " + parsedData[1] + "\nmotorRight: " + parsedData[2])

    def rosspin(self):
        rospy.spin()

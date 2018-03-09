#!/usr/bin/env python
import LocationAudio
import gui
import rospy
from std_msgs.msg import String
from threading import Thread
from dronekit import connect,LocationGlobal,VehicleMode, Command, LocationGlobalRelative

isObstacle = 0
motorLeft = 1500
motorRight = 1500

targetbuilding = ""
targetbuildingindex = -1
returntrip = False

navigator = 0

def callback(data):
    global isObstacle
    global motorLeft
    global motorRight
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    parsedData = data.data.split(" ")
    isObstacle = int(parsedData[0])
    motorLeft = int(parsedData[1])
    motorRight = int(parsedData[2])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("avoid", String, callback)
    rospy.spin()

def run_mission():
    global navigator
    global isObstacle
    global motorLeft
    global motorRight
    navigator.vehicle.commands._vehicle._current_waypoint = 2
    navigator.vehicle.commands.next = 2
    navigator.vehicle.mode = VehicleMode("AUTO")
    missionsize = navigator.vehicle.commands.count
    print ("missionsize: " + str(missionsize))
    lastwaypoint = -1
    while True:
        nextwaypoint = navigator.vehicle.commands.next
        # print('Distance to waypoint (%s): %s' % (nextwaypoint, navigator.distance_to_current_waypoint()))
        if nextwaypoint != lastwaypoint:
            print ("At " + str(nextwaypoint))
            lastwaypoint = nextwaypoint
        if nextwaypoint == missionsize:
            break
        if (isObstacle == 1):
            navigator.pause_mission()
            print("obstacle detected")
            while isObstacle:
                navigator.overwriteChannel(1, motorLeft)
                navigator.overwriteChannel(3, motorRight)
            print("obstacle clear")
            navigator.clearoverwrites()
            navigator.continue_mission()
    navigator.vehicle.mode = VehicleMode("GUIDED")

def mainfunction():
    global navigator
    global isObstacle
    global motorLeft
    global motorRight
    navigator = LocationAudio.Navigator(connection_string="/dev/ttyACM0", baudrate=115200)
    while True:
        ui = gui.UI()
        # User Interface
        # Get User click on map to generate destination
        # Waypoint generate & upload to Ardurover
        ui.mainloop()
        targetbuilding = ui.location.get()
        targetbuildingindex = ui.locationindex.get()

        # ROS
        # Get obstacles
        # If too close --> Manual mode w/ pwm instructions
        # If far enough --> Auto
        # Ardurover GPS
        navigator.setLocationindex(targetbuildingindex)
        print ("The target location is: " + navigator.gettargetlocation())

        currentmission = navigator.getcurrentmission()
        navigator.upload_mission(currentmission)

        # Need to integrate mission control with obstacle avoidance

        # while no obstacle detected --> run automated mission control
        run_mission()

        currentaudio = navigator.getaudio()
        navigator.PlayAudio(currentaudio)  # Plays audio file at current location

        # Need to start return mission whose file is located at current mission + 6
        navigator.setLocationindex(targetbuildingindex + 6)
        currentmission = navigator.getcurrentmission()

        # while no obstacle detected --> run automated mission control
        navigator.upload_mission(currentmission)
        run_mission()

        # if obstacle detected,
        # navigator.pause_mission()
        # recheck for obstacle and when its clear,
        # navigator.continue_mission()

        # Restart a new mission by looping back to top, while loop

if __name__ == "__main__":
    # Main function
    Thread(target = mainfunction).start()
    listener()

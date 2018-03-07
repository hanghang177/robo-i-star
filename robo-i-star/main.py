import LocationAudio
import gui

targetbuilding = ""
targetbuildingindex = -1
returntrip = False

if __name__ == "__main__":
    # Main function
    navigator = LocationAudio.Navigator(connection_string="", baudrate=57600)
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
        navigator.run_mission()

        currentaudio = navigator.getaudio()
        navigator.PlayAudio(currentaudio)  # Plays audio file at current location

        # Need to start return mission whose file is located at current mission + 6
        navigator.setLocationindex(targetbuildingindex + 6)
        currentmission = navigator.getcurrentmission()

        # while no obstacle detected --> run automated mission control
        navigator.upload_mission(currentmission)
        navigator.run_mission()

        # if obstacle detected,
        # navigator.pause_mission()
        # recheck for obstacle and when its clear,
        # navigator.continue_mission()

        # Restart a new mission by looping back to top, while loop

import dronekit
import pygame
import LocationAudio
import gui

targetbuilding = ""
targetbuildingindex = -1
returntrip = False

if __name__ == "__main__":
    # Main function
    navigator = LocationAudio.Navigator()
    while True:
        ui = gui.UI()
        # User Interface
            # Get User click on map to generate destination
            # Waypoint generate & upload to Ardurover
        ui.mainloop()
        targetbuilding =  ui.location.get()
        targetbuildingindex = ui.locationindex.get()

        # ROS
            # Get obstacles
            # If too close --> Manual mode w/ pwm instructions
            # If far enough --> Auto
        # Ardurover GPS
        navigator.setLocationindex(targetbuildingindex)
        print ("The target location is: " + navigator.gettargetlocation())

        navigator.upload_mission(navigator.missionfiles[targetbuildingindex])
        navigator.run_mission()
        navigator.PlayAudio(navigator.audio[targetbuildingindex])
        navigator.upload_mission(navigator.missionfiles[targetbuildingindex+6])
        navigator.run_mission()

        print ('mission finished')

            #Ardurover Get Location(Waypoint)
            #Ardurover Play Audio
        # GPS Navigation
            # Done in Ardurover

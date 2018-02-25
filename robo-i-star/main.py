import dronekit
import pygame
import LocationAudio
import gui

targetbuilding = ""
returntrip = False

if __name__ == "__main__":
    # Main function
    #robot = LocationAudio.Navigator()
    ui = gui.UI()
    # User Interface
        # Get User click on map to generate destination
        # Waypoint generate & upload to Ardurover
    ui.mainloop()
    targetbuilding =  ui.location.get()
    print targetbuilding
    # ROS
        # Get obstacles
        # If too close --> Manual mode w/ pwm instructions
        # If far enough --> Auto

    # Ardurover GPS
        #Ardurover Get Location(Waypoint)
        #Ardurover Play Audio

    # GPS Navigation
        # Done in Ardurover

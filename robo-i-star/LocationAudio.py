from dronekit import connect,LocationGlobal,VehicleMode
import pygame

"""
TODO: 
* file io for initilizer when give waypoint file--will waypoint files have waypoint name as well?
* find out how to initialize audio file array (probs more file io)
* create a function that can go to a given waypoint and back to home
"""

class Navigator:
    def __init__(self, fwaypoints):#if given a file of waypoints this will initialize the waypoints using the file
        connection_string = "COM6"
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, baud=57600, wait_ready=False)

        # locations, their audios and their waypoints are stored in parallel arrays
        self.locations = []
        self.audio = []
        self.waypoints = []  # note it's in the format latitude,longitude
        currentAlt = self.vehicle.location.global_frame.alt

    def __init__(self):#if no file is given, this will simply initialize the waypoints to what I originally found on google
        connection_string = "COM6"
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, baud=57600, wait_ready=False)

        # locations, their audios and their waypoints are stored in parallel arrays
        self.locations = ["Grainger", "Talbot", "Everitt", "Engineering Hall", "Material Science Building",
                     "Mechanical Engineering Building"]
        self.audio = []
        self.waypoints = []  # note it's in the format latitude,longitude
        # note these waypoints were taken from google maps locations right infront of the building--they can and should be adjusted if necessary
        currentAlt= self.vehicle.location.global_frame.alt
        self.waypoints.append(LocationGlobal(-88.226958, 40.112232, currentAlt))  # Grainger
        self.waypoints.append(LocationGlobal(-88.227616, 40.111863, currentAlt))  # Talbot
        self.waypoints.append(LocationGlobal(-88.227629, 40.111151, currentAlt))  # Everitt
        self.waypoints.append(LocationGlobal(-88.226949, 40.111126, currentAlt))  # Engineering Hall
        self.waypoints.append(LocationGlobal(-88.226505, 40.111128, currentAlt))  # Material Science Building
        self.waypoints.append(LocationGlobal(-88.226678, 40.111692, currentAlt))  # Mechanical Engineering Building

    def GoToAll(self):
        #this function goes through all the waypoints
        self.vehicle.mode = VehicleMode("Guided")
        self.vehicle.armed = True

        for x in range(len(self.waypoints)):
            while (self.waypoints[x] != currentloc):  # this loop should take the robot to the location it needs to go
                lat = (self.vehicle.location.global_frame.lat)
                lon = (self.vehicle.location.global_frame.lon)
                currentloc = LocationGlobal(lat, lon, self.vehicle.location.global_frame.alt)
                LocationGlobal()
                self.vehicle.simple_goto(self.waypoints[x])
            # after it gets to the desired location it should play the audio

        self.PlayAudio(self.audio[x])

        self.vehicle.mode = VehicleMode("RTL")  # after the robot has gone through all the waypoints this should bring it back to its original location

    def PlayAudio(self,audiofile):#this function is used to play audio
        pygame.mixer.init()
        BUFFER = 3072  # audio buffer size, number of samples since pygame 1.8.
        FREQ, SIZE, CHAN = pygame.mixer.get_init()
        pygame.mixer.init(FREQ, SIZE, CHAN, BUFFER)

        clock = pygame.time.Clock()
        pygame.mixer.music.load(audiofile)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            print "Playing..."
            clock.tick(1000)



"""this is test code for the audio
robo = Navigator()
robo.initMixer()
robo.PlayAudio('./tests/SampleAudio_0.4mb.mp3')
"""
from dronekit import connect,LocationGlobal,VehicleMode

def PlayAudio(audiofile):#this function is used to play audio
    import pygame
    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load(audiofile)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

connection_string = "COM6"

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, baud=57600, wait_ready=False)

#locations, their audios and their waypoints are stored in parallel arrays
locations =  ["Grainger","Talbot","Everitt","Engineering Hall","Material Science Building","Mechanical Engineering Building"]
audio = []
waypoints = []#note it's in the format latitude,longitude
#note these waypoints were taken from google maps locations right infront of the building--they can and should be adjusted if necessary
waypoints.append(LocationGlobal(-88.226958, 40.112232 , vehicle.location.global_frame.alt))#Grainger
waypoints.append(LocationGlobal( -88.227616, 40.111863 , vehicle.location.global_frame.alt))#Talbot
waypoints.append(LocationGlobal(-88.227629, 40.111151, vehicle.location.global_frame.alt))#Everitt
waypoints.append(LocationGlobal(-88.226949, 40.111126, vehicle.location.global_frame.alt))#Engineering Hall
waypoints.append(LocationGlobal(-88.226505, 40.111128, vehicle.location.global_frame.alt))#Material Science Building
waypoints.append(LocationGlobal(-88.226678, 40.111692, vehicle.location.global_frame.alt))#Mechanical Engineering Building


for x in range(len(waypoints)):
    vehicle.mode = VehicleMode("Guided")
    while (waypoints[x]!=currentloc): #this loop should take the robot to the location it needs to go
        lat = (vehicle.location.global_frame.lat)
        lon = (vehicle.location.global_frame.lon)
        currentloc = LocationGlobal(lat, lon, vehicle.location.global_frame.alt)
        LocationGlobal()
        vehicle.simple_goto(waypoints[x])
    #after it gets to the desired location it should play the audio
    PlayAudio(audio[x])

vehicle.mode=VehicleMode("RTL") #after the robot has gone through all the waypoints this should bring it back to its original location




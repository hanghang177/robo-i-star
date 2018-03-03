from dronekit import connect,LocationGlobal,VehicleMode, Command, LocationGlobalRelative
import pygame
import time
import math

#connection_string = "COM6"
connection_string = ""
baudrate = 57600

class Navigator:
    def __init__(self): #Navigator program initialization
        global connection_string
        global baudrate

        if not connection_string:
            import dronekit_sitl
            sitl = dronekit_sitl.start_default(lat= 40.111718, lon= -88.227020)
            connection_string = sitl.connection_string()

        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, baud=baudrate, wait_ready=False)

        # Audio, locations, and mission files are stored in parallel arrays
        self.locationindex = -1
        self.locations = ["Grainger", "Talbot", "Everitt", "Engineering Hall", "Material Science Building",
                     "Mechanical Engineering Lab"]
        self.audio = ["audio/Grainger.mp3", "audio/Talbot.mp3", "audio/Everitt.mp3", "audio/Engineering Hall.mp3", "audio/Material Science Building.mp3",
                     "audio/Mechanical Engineering Lab.mp3"]
        self.missionfiles = ["mission/waypoints.waypoints","mission/talbot_lab_forward.waypoints","mission/everitt_lab__forward.waypoints","mission/engineering_hall_forward.waypoints",
                            "mission/material_science_forward.waypoints","mission/mechanical_lab_forward.waypoints","mission/grainger_backwards.waypoints","mission/talbot_lab_backwards.waypoints",
                            "mission/everitt_lab_backwards.waypoints","mission/engineering_hall_backwards.waypoints","mission/material_science_backwards.waypoints","mission/mechanical_lab_backwards.waypoints"]

        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        self.vehicle.simple_takeoff(10)
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= 10 * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)

    def PlayAudio(self,audiofile): #this function is used to play audio
        pygame.mixer.init()
        BUFFER = 3072  # audio buffer size, number of samples since pygame 1.8.
        FREQ, SIZE, CHAN = pygame.mixer.get_init()
        pygame.mixer.init(FREQ, SIZE, CHAN, BUFFER)

        clock = pygame.time.Clock()
        pygame.mixer.music.load(audiofile)
        pygame.mixer.music.play()
        print ("Playing...")
        while pygame.mixer.music.get_busy():
            clock.tick(1000)

    def get_location_metres(self,original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        return LocationGlobal(newlat, newlon, original_location.alt)

    def get_distance_metres(self,aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def distance_to_current_waypoint(self):
        """
        Gets distance in metres to the current waypoint.
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = self.vehicle.commands.next
        if nextwaypoint == 0:
            return None
        missionitem = self.vehicle.commands[nextwaypoint - 1]  # commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat, lon, alt)
        distancetopoint = self.get_distance_metres(self.vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint

    def readmission(self,aFileName):
        """
        Load a mission from a file into a list. The mission definition is in the Waypoint file
        format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).

        This function is used by upload_mission().
        """
        print ("\nReading mission from file: %s" % aFileName)
        cmds = self.vehicle.commands
        missionlist = []
        with open(aFileName) as f:
            for i, line in enumerate(f):
                if i == 0:
                    if not line.startswith('QGC WPL 110'):
                        raise Exception('File is not supported WP version')
                else:
                    linearray = line.split('\t')
                    ln_index = int(linearray[0])
                    ln_currentwp = int(linearray[1])
                    ln_frame = int(linearray[2])
                    ln_command = int(linearray[3])
                    ln_param1 = float(linearray[4])
                    ln_param2 = float(linearray[5])
                    ln_param3 = float(linearray[6])
                    ln_param4 = float(linearray[7])
                    ln_param5 = float(linearray[8])
                    ln_param6 = float(linearray[9])
                    ln_param7 = float(linearray[10])
                    ln_autocontinue = int(linearray[11].strip())
                    cmd = Command(0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2,
                                  ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                    missionlist.append(cmd)
        return missionlist

    def upload_mission(self,aFileName):
        """
        Upload a mission from a file.
        """
        # Read mission from file
        missionlist = self.readmission(aFileName)
        # Clear existing mission from vehicle
        cmds = self.vehicle.commands
        cmds.clear()
        # Add new mission to vehicle
        for command in missionlist:
            cmds.add(command)
        print (' Upload mission')
        self.vehicle.commands.upload()

    def run_mission(self):
        self.vehicle.commands._vehicle._current_waypoint = 2
        self.vehicle.commands.next = 2
        self.vehicle.mode = VehicleMode("AUTO")
        missionsize = self.vehicle.commands.count
        print ("missionsize: " + str(missionsize))
        lastwaypoint = -1
        while True:
            nextwaypoint = self.vehicle.commands.next
            print('Distance to waypoint (%s): %s' % (nextwaypoint, self.distance_to_current_waypoint()))
            if nextwaypoint != lastwaypoint:
                print ("At " + str(nextwaypoint))
                lastwaypoint = nextwaypoint
            if nextwaypoint == missionsize:
                break
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")

    def pause_mission(self):
        self.vehicle.mode = VehicleMode("GUIDED")

    def continue_mission(self):
        self.vehicle.mode = VehicleMode("AUTO")
        missionsize = self.vehicle.commands.count
        while True:
            nextwaypoint = self.vehicle.commands.next
            if nextwaypoint == missionsize - 1:
                break
            time.sleep(1)
        self.vehicle.mode = VehicleMode("GUIDED")

    def setLocationindex(self, locationindex):
        self.locationindex = locationindex

    def gettargetlocation(self):
        return self.locations[self.locationindex]

    def getaudio(self):
        return self.audio[self.locationindex]

    def getcurrentmission(self):
        return self.missionfiles[self.locationindex]

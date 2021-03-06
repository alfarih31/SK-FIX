from time import sleep
import logging
import dronekit

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = dronekit.Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist

def upload_mission(aFileName):
    """
    Upload a mission from a file.
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

vehicle = dronekit.connect("/dev/ttyUSB0", baud=57600)
print("CONNECTED")

while not vehicle.is_armable:
    print("WAITING INITIALIZING")
    sleep(1)

print(vehicle.system_status.state)
vehicle.mode = dronekit.VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("WAITING FOR ARMED")
    sleep(500)

vehicle.simple_takeoff(alt=20.0)
while True:
    print('Altitude: %d'%vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= 20 * 0.95:
        logging.info('REACHED TARGET ALTITUDE')
        break
    sleep(500)

upload_mission("misi1.waypoints")
print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0
vehicle.mode = dronekit.VehicleMode("AUTO")

while True:
    nextwaypoint=vehicle.commands.next
    print("Current wp %d"%nextwaypoint)
    sleep(1)

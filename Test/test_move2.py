from time import sleep, time
import math
import dronekit
from pymavlink import mavutil

print("HERE")
vehicle = dronekit.connect("/dev/ttyUSB0", baud=57600, wait_ready=False)
print("CONNECTED")

# while not vehicle.is_armable:
#     print("WAITING INITIALIZING")
#     sleep(1)

vehicle.mode = dronekit.VehicleMode("GUIDED")
print(vehicle.mode)
vehicle.armed = True
while not vehicle.armed:
    vehicle.armed = True
    print("WAITING FOR ARMED")
    sleep(1)

vehicle.mode = dronekit.VehicleMode("GUIDED")
#vehicle.airspeed = 5
vehicle.simple_takeoff(alt=1)
sleep(5)
# alt_global = vehicle.location.global_frame.alt
# print(alt_global)
# alt = vehicle.location.global_relative_frame.alt
# while alt < 0.8:
#     alt = vehicle.location.global_relative_frame.alt
#     alt_global = vehicle.location.global_frame.alt
#     print(alt_global)
#     print('Altitude: %.2f'%vehicle.location.global_relative_frame.alt)
#     sleep(0.5)

def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def get_point_distance(point1, point2):
    # approximate radius of earth in km
    R = 6378137.0

    lat1 = math.radians(point1[0])
    lon1 = math.radians(point1[1])
    lat2 = math.radians(point2[0])
    lon2 = math.radians(point2[1])

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance

def llg_from_distance(d, lat, lon, brng):
    R = 6378137.0 #Radius of the Earth in Meters
    brng = math.radians(brng)

    lat = math.radians(lat) #Current lat point converted to radians
    lon = math.radians(lon) #Current long point converted to radians

    lat2 = math.asin(math.sin(lat)*math.cos(d/R) +
                     math.cos(lat)*math.sin(d/R)*math.cos(brng))

    lon2 = lon + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat),
                            math.cos(d/R)-math.sin(lat)*math.sin(lat2))

    lat2 = math.degrees(lat2)
    lon2 = math.degrees(lon2)

    return (lat2, lon2)

def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time()
    while time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# print("MISI YAWING 45")
# heading = vehicle.heading
# #target_heading = heading + 45
# #@current_yaw = math.degrees(vehicle.attitude.yaw)
# current_yaw = heading
# target_heading = current_yaw + 45
# send_attitude_target(yaw_angle=target_heading)
# d_yaw = abs(target_heading-current_yaw)
# print("Current heading: %d, target: %d"%(heading, target_heading))
# while d_yaw > 5:
#     heading = vehicle.heading
#     #current_yaw = math.degrees(vehicle.attitude.yaw)
#     current_yaw = heading
#     print("Current heading: %d, target: %d, Yaw: %.2f"%(heading, target_heading, current_yaw))
#     d_yaw = abs(target_heading-current_yaw)
#     attitude = vehicle.attitude
#     pitch = math.degrees(attitude.pitch)
#     roll = math.degrees(attitude.roll)
#     send_attitude_target(yaw_angle=target_heading, pitch_angle=pitch, roll_angle=roll)
#     sleep(0.1)

# print("MISI YAW")
# heading = vehicle.heading
# print(heading)
# target_heading = heading + 45
# condition_yaw(target_heading)
# sleep(4)

# print("HOVER 3 Detik 1")
# set_attitude(duration=2)

# print("MISI MAJU NO GPS")
# start_lat = vehicle.location.global_relative_frame
# print(start_lat)
# set_attitude(pitch_angle = -10, thrust = 0.5, duration = 4, yaw_angle=vehicle.heading)
# end_lat = vehicle.location.global_relative_frame
# print(end_lat)

print("HOVER 3 Detik 2")
set_attitude(duration=2)

print("MISI MAJU 2M")
location = vehicle.location.global_relative_frame
heading = vehicle.heading
while not heading:
    heading = vehicle.heading
    print(heading)
    sleep(0.5)
targetDistance = 1

target = llg_from_distance(1, location.lat, location.lon, heading)

lat_target, lon_target = target
vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1))
while vehicle.mode.name == "GUIDED":
    location = vehicle.location.global_relative_frame
    remainingDistance = get_point_distance((lat_target, lon_target), (location.lat, location.lon))
    print("Distance to target: %.3f, Heading %d, Target %.2f"%(remainingDistance, vehicle.heading,vehicle.location.global_frame.alt))
    if remainingDistance <= targetDistance*0.3: #Just below target, in case of undershoot.
        print("Reached target")
        break
    elif remainingDistance >= targetDistance*0.3:
        vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1))
    sleep(0.5)


condition_yaw(90)
sleep(3)

print("MISI MAJU 2M")
location = vehicle.location.global_relative_frame
heading = vehicle.heading
targetDistance = 3
target = llg_from_distance(3, location.lat, location.lon, heading)
lat_target, lon_target = target
vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
while vehicle.mode.name == "GUIDED":
    location = vehicle.location.global_relative_frame
    remainingDistance = get_point_distance((lat_target, lon_target), (location.lat, location.lon))
    print("Distance to target: %.3f, Heading %d, Target %.2f"%(remainingDistance, vehicle.heading,vehicle.location.global_frame.alt))
    if remainingDistance <= targetDistance*0.25: #Just below target, in case of undershoot.
        print("Reached target")
        break
    elif remainingDistance >= targetDistance*0.3:
        send_attitude_target(yaw_angle=heading+90)
        vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
    sleep(0.5)


print("HOVER 3 Detik 3")
set_attitude(duration=2)

print("MISI MAJU 3M")
location = vehicle.location.global_relative_frame
heading = vehicle.heading
targetDistance = 3

condition_yaw(180)
sleep(3)

location = vehicle.location.global_relative_frame
heading = vehicle.heading
target = llg_from_distance(3, location.lat, location.lon, heading)
lat_target, lon_target = target
vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
while vehicle.mode.name == "GUIDED":
    location = vehicle.location.global_relative_frame
    remainingDistance = get_point_distance((lat_target, lon_target), (location.lat, location.lon))
    print("Distance to target: %.3f, Heading %d, Target %.2f"%(remainingDistance, vehicle.heading,vehicle.location.global_frame.alt))
    if remainingDistance <= targetDistance*0.25: #Just below target, in case of undershoot.
        print("Reached target")
        break
    elif remainingDistance >= targetDistance*0.3:
        send_attitude_target(yaw_angle=heading+90)
        vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
    sleep(0.5)

set_attitude(duration=2)

print("MISI MAJU 3M")
location = vehicle.location.global_relative_frame
heading = vehicle.heading
targetDistance = 3

condition_yaw(90)
sleep(3)

location = vehicle.location.global_relative_frame
heading = vehicle.heading
target = llg_from_distance(3, location.lat, location.lon, heading)
lat_target, lon_target = target
vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
while vehicle.mode.name == "GUIDED":
    location = vehicle.location.global_relative_frame
    remainingDistance = get_point_distance((lat_target, lon_target), (location.lat, location.lon))
    print("Distance to target: %.3f, Heading %d, Target %.2f"%(remainingDistance, vehicle.heading,vehicle.location.global_frame.alt))
    if remainingDistance <= targetDistance*0.25: #Just below target, in case of undershoot.
        print("Reached target")
        break
    elif remainingDistance >= targetDistance*0.3:
        send_attitude_target(yaw_angle=heading+90)
        vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=1.5))
    sleep(0.5)

set_attitude(duration=3)
print("LANDING")
vehicle.mode = dronekit.VehicleMode("LAND")
for _ in range(2):
    vehicle.mode = dronekit.VehicleMode("LAND")
    sleep(0.2)

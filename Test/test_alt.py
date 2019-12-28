from time import sleep, time
import math
import dronekit
from pymavlink import mavutil

print("HERE")
vehicle = dronekit.connect("127.0.0.1:14550", baud=57600, wait_ready=False)
print("CONNECTED")

while not vehicle.is_armable:
    print("WAITING INITIALIZING")
    sleep(1)
vehicle.mode = dronekit.VehicleMode("GUIDED")
print(vehicle.mode)
vehicle.armed = True
while not vehicle.armed:
    vehicle.armed = True
    print("WAITING FOR ARMED")
    sleep(1)

vehicle.mode = dronekit.VehicleMode("GUIDED")
#vehicle.airspeed = 5
vehicle.wait_simple_takeoff(alt=1)

print('Altitude: %d'%vehicle.location.global_relative_frame.alt)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
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

heading = vehicle.heading
#target_heading = heading + 45
#@current_yaw = math.degrees(vehicle.attitude.yaw)
current_yaw = heading
target_heading = current_yaw + 45
send_attitude_target(yaw_angle=target_heading)
d_yaw = abs(target_heading-current_yaw)
print("Current heading: %d, target: %d"%(heading, target_heading))
while d_yaw > 5:
    heading = vehicle.heading
    #current_yaw = math.degrees(vehicle.attitude.yaw)
    current_yaw = heading
    print("Current heading: %d, target: %d, Yaw: %.2f"%(heading, target_heading, current_yaw))
    d_yaw = abs(target_heading-current_yaw)
    attitude = vehicle.attitude
    pitch = math.degrees(attitude.pitch)
    roll = math.degrees(attitude.roll)
    send_attitude_target(yaw_angle=target_heading, pitch_angle=pitch, roll_angle=roll)
    sleep(0.1)

#set_attitude(duration=3, yaw_angle=vehicle.heading)

start_lat = vehicle.location.global_relative_frame
print(start_lat)
#set_attitude(pitch_angle = -5, thrust = 0.5, duration = 1, yaw_angle=vehicle.heading)
end_lat = vehicle.location.global_relative_frame
print(end_lat)

#set_attitude(duration=3, yaw_angle=vehicle.heading)

# heading = vehicle.heading
# target_heading = heading + 45
# send_attitude_target(yaw_angle=target_heading)
# current_yaw = math.degrees(vehicle.attitude.yaw)
# print("Current heading 2: %d, target: %d, Yaw: %.2f"%(heading, target_heading, current_yaw))
# d_yaw = abs(90-current_yaw)
# while d_yaw > 1e-1:
#     heading = vehicle.heading
#     current_yaw = math.degrees(vehicle.attitude.yaw)
#     print("Current heading 2: %d, target: %d, Yaw: %.2f"%(heading, target_heading, current_yaw))
#     d_yaw = abs(90-current_yaw)
#     #send_attitude_target(yaw_angle=target_heading)
#     sleep(0.1)

#set_attitude(duration=3, yaw_angle=vehicle.heading)

start_lat = vehicle.location.global_relative_frame
print(start_lat)
#set_attitude(pitch_angle = -5, thrust = 0.5, duration = 1, yaw_angle=vehicle.heading)
end_lat = vehicle.location.global_relative_frame
print(end_lat)

#set_attitude(duration=3, yaw_angle=vehicle.heading)

vehicle.mode = dronekit.VehicleMode("LAND")

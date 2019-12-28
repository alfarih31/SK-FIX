from time import sleep, time
from os import getenv
import math

from dotenv import load_dotenv
from sensor_msgs.msg import CompressedImage
from pymavlink import mavutil
from turbojpeg import TurboJPEG, TJPF_BGR

import dronekit
import rospy
import cv2
import cv_bridge

import grpc

from mission_msg.msg import Mission

import SK_pb2
import SK_pb2_grpc

load_dotenv()

class ARRGDrone:
    def __init__(self):
        self.turbo = TurboJPEG(lib_path="/usr/lib/libturbojpeg.so")

        rospy.init_node("NODE", anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.pub_image = rospy.Publisher("/image", CompressedImage, queue_size=1)
        self.pub_mission = rospy.Publisher("/mission", Mission, queue_size=1)
        self.cam = cv2.VideoCapture(int(getenv("WEBCAM")))
        print("OPENING CAMERA")
        while not self.cam.isOpened():
            pass

        self.channel = grpc.insecure_channel("%s:%d"%(getenv("SERVER_ADDRESS"), int(getenv("SERVER_PORT"))))
        self.stub = SK_pb2_grpc.WSStub(self.channel)

        self.board = getenv("BOARD")
        self.baud = int(getenv("BAUDRATE"))
        print("BOARD: %s BAUD: %d"%(self.board, self.baud))
        self.vehicle = dronekit.connect(self.board, baud=self.baud, wait_ready=False)
        print("CONNECTED")
        while not self.vehicle.is_armable:
            print("WAITING INITIALIZING")
            sleep(1)
        self.init_drone()

        self.points = []
        self.distance = 0
        self.point = (0, 0, 0, 0, 0) # Lat, Lon, Alt, Head, Distance
        self.on_mission = False
        self.on_yaw = False
        self.seq = 0
        self.attempt = 0
        self.num_of_detected = 4

        self.DEFAULT_TAKEOFF_THRUST = 0.7
        self.SMOOTH_TAKEOFF_THRUST = 0.6
        self.responses_loop()

    def init_drone(self):
        lat = self.vehicle.location.global_relative_frame.lat
        while not lat:
            lat = self.vehicle.location.global_relative_frame
            print("WAITING FOR GPS TO LOCK")
            sleep(1)
        self.vehicle.mode = dronekit.VehicleMode("GUIDED_NOGPS")

        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("WAITING FOR ARMED")
            sleep(1)

        aTargetAltitude = 1.5
        thrust = self.DEFAULT_TAKEOFF_THRUST
        while True:
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(" Altitude: %f  Desired: %f"%(current_altitude, aTargetAltitude))
            if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
                print("Reached target altitude")
                break
            elif current_altitude >= aTargetAltitude*0.6:
                thrust = self.SMOOTH_TAKEOFF_THRUST
            self.set_attitude(thrust=thrust)
            sleep(0.2)

    def get_bearing(self, aLocation1, aLocation2):
        off_x = aLocation2[1] - aLocation1[1]
        off_y = aLocation2[0] - aLocation1[0]
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def llg_from_distance(self, distance, lat, lon, brng):

        R = 6378.1 #Radius of the Earth
        d = distance/1e+3

        lat = math.radians(lat) #Current lat point converted to radians
        lon = math.radians(lon) #Current long point converted to radians

        lat2 = math.asin(math.sin(lat)*math.cos(d/R) +
                         math.cos(lat)*math.sin(d/R)*math.cos(brng))

        lon2 = lon + math.atan2(math.sin(brng)*math.sin(d/R)*math.cos(lat),
                                math.cos(d/R)-math.sin(lat)*math.sin(lat2))

        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)

        return (lat2, lon2)

    def prepare_mission(self, img):
        print("QUAD YAWING")
        target_heading = self.vehicle.heading+45
        self.set_yaw(target_heading)
        sleep(0.5)

        d_yaw = abs(target_heading-self.vehicle.heading)
        loop_count = 0
        while d_yaw > 5:
            if loop_count >= 30:
                break
            d_yaw = abs(target_heading-self.vehicle.heading)
            sleep(0.1)
            loop_count += 1

        if self.attempt >= 7:
            location = self.vehicle.location.global_relative_frame
            lat, lon = self.llg_from_distance(1.5, location.lat, location.lon, self.vehicle.heading)
            self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat, lon, 1))
            sleep(0.5)
            print("QUAD MOVING")

            location = self.vehicle.location.global_relative_frame
            distance = self.get_point_distance((lat, lon), (location.lat, location.lon))
            while abs(distance) > 0.4:
                location = self.vehicle.location.global_relative_frame
                distance = self.get_point_distance((lat, lon), (location.lat, location.lon))
                sleep(0.1)
            self.attempt = 0

        img_bytes = self.turbo.encode(img, quality=80, pixel_format=TJPF_BGR)
        request = SK_pb2.fromClient(on_mission=False, image=img_bytes)
        response = self.stub.clientStream(request)
        self.distance = response.distance

        location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading
        if self.distance:
            target_llg = self.llg_from_distance(self.distance, location.lat, location.lon, heading)
            status = self.check_point(target_llg)
            if status:
                self.point = (target_llg[0], target_llg[1], heading, distance)
                return True
            else:
                return False
        else:
            return False

    def start_mission(self):
        self.on_yaw = True
        print("GO TO POINT")
        self.register_point()
        self.on_mission = True

    def register_point(self):
        self.points.append(self.point)
        stamp = rospy.Time.now()
        mission = Mission()
        mission.header.stamp = stamp
        mission.header.seq = self.seq
        mission.latitude = self.point[0]
        mission.longitude = self.point[1]
        mission.altitude = self.point[2]
        mission.heading = self.point[3]
        mission.distance = self.point[4]

        self.seq += 1
        self.pub_mission.publish(mission)

    def get_point_distance(self, point1, point2):
        # approximate radius of earth in km
        R = 6373.0

        lat1 = math.radians(point1[0])
        lon1 = math.radians(point1[1])
        lat2 = math.radians(point2[0])
        lon2 = math.radians(point2[1])

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = (R * c)*1e+3
        return distance

    def check_point(self, new_point):
        distance = self.get_point_distance(self.point, new_point)
        if abs(distance) >= 1:
            return True
        else:
            return False

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                             thrust=0.5):
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.vehicle.attitude.yaw
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                     thrust=0.5, duration=0):
        self.send_attitude_target(roll_angle, pitch_angle,
                                  yaw_angle, yaw_rate, False,
                                  thrust)
        start = time()
        while time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                      yaw_angle, yaw_rate, False,
                                      thrust)
            sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                                  0, 0, True,
                                  thrust)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
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

    def set_yaw(self, heading):
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            0, # param 4, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        self.vehicle.send_mavlink(msg)
        self.attempt += 1

    def monitor_mission_status(self):
        location = self.vehicle.location.global_relative_frame
        if self.on_yaw:
            target_heading = self.get_bearing([location.lat, location.lon], self.point)
            heading = self.vehicle.heading
            print("Current heading: %d, target: %d"%(heading, target_heading))
            d_yaw = abs(target_heading - heading)
            if d_yaw < 5:
                self.on_yaw = False
            else:
                self.set_yaw(target_heading)
        else:
            self.set_attitude(pitch_angle=-5, duration=0.5)
            distance = self.get_point_distance((location.lat, location.lon), self.point)
            print("Current Distance to target: %d"%distance)
            if abs(distance) < 0.5:
                self.on_mission = False
                self.attempt = 0
                self.num_of_detected += 1

    def prepare_mission_first(self):
        image = self.camera_callback()
        image_msg = self.bridge.cv2_to_compressed_imgmsg(image)
        self.pub_image.publish(image_msg)
        if not self.on_mission:
            stat = self.prepare_mission(image)
            if stat:
                self.start_mission()
        else:
            self.monitor_mission_status()
            img_bytes = self.turbo.encode(image)
            request = SK_pb2.fromClient(on_mission=True, image=img_bytes)
            _ = self.stub.clientStream(request)

    def camera_callback(self):
        ret, img = self.cam.read()
        if not ret:
            while True:
                ret, img = self.cam.read()
                if not ret:
                    continue
                break
        return img

    def responses_loop(self):
        print("START LOOP")
        while True:
            try:
                if self.num_of_detected >= 4:
                    break
                self.prepare_mission_first()
            except KeyboardInterrupt:
                self.channel.close()
                break
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        alt = self.vehicle.location.global_relative_frame.alt
        while alt > 0:
            alt = self.vehicle.location.global_relative_frame
            sleep(0.1)
        self.vehicle.armed = False

if __name__ == "__main__":
    try:
        drone = ARRGDrone()
#        drone.responses_loop()
    except KeyboardInterrupt:
        drone.vehicle.close()

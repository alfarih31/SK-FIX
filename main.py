from time import sleep, time
from os import getenv
import math

from dotenv import load_dotenv
from sensor_msgs.msg import CompressedImage
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
#        while not self.vehicle.is_armable:
#            print("WAITING INITIALIZING")
#            sleep(1)
        self.init_drone()

        self.points = []
        self.distance = 0
        self.point = (0, 0, 0, 0, 0) # Lat, Lon, Alt, Head, Distance
        self.on_mission = False
        self.seq = 0
        self.attempt = 0
        self.num_of_detected = 4
        self.responses_loop()

    def init_drone(self):
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.vehicle.armed = True
            print("WAITING FOR ARMED")
            sleep(1)
        self.vehicle.simple_takeoff(alt=1)
        while True:
            print('Altitude: %d'%self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= 1 * 0.95:#
                print('Reached Target Altitude')
                break
            self.vehicle.simple_takeoff(alt=1)
            sleep(0.1)

    def llg_from_distance(self, d, lat, lon, brng):

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

    def maintain_yaw(self):
        print("Maintain Yawing")
        heading = self.vehicle.heading
        target_heading = heading + 45
        if target_heading > 360:
            target_heading -= 360
        print("Current heading: %d, target: %d"%(heading, target_heading))
        d_yaw = abs(target_heading-heading)
        self.send_attitude_target(yaw_angle=target_heading)
        while d_yaw > 5:
            heading = self.vehicle.heading
            print("Current heading: %d, target: %d"%(heading, target_heading))
            d_yaw = abs(target_heading-heading)
            self.send_attitude_target(yaw_angle=target_heading)
            sleep(0.1)
        self.attempt += 1

    def request_to_server(self, img):
        img_bytes = self.turbo.encode(img, quality=80, pixel_format=TJPF_BGR)
        request = SK_pb2.fromClient(on_mission=False, image=img_bytes)
        response = self.stub.clientStream(request)
        return response

    def prepare_mission(self):
        self.maintain_yaw()

        if self.attempt >= 7:
            print("Quad move forward")
            location = self.vehicle.location.global_relative_frame
            lat, lon = self.llg_from_distance(1.5, location.lat, location.lon, self.vehicle.heading)
            self.goto((location.lat, location.lon, location.alt), (lat, lon, location.alt))
            sleep(0.5)
            self.attempt = 0

        img = self.capture_camera()
        response = self.request_to_server(img)
        self.distance = response.distance
        self.rect_and_pub_image(img, response.distance, response.tx, response.ty, response.bx, response.by)

        location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading
        if self.distance:
            llg = self.llg_from_distance(self.distance, location.lat, location.lon, heading)
            status = self.check_point(llg)
            if status:
                self.point = (llg[0], llg[1], heading, self.distance)
                return True
            else:
                return False
        else:
            return False

    def goto(self, ref, target):
        lat_target, lon_target, alt_target = target
        targetDistance = self.get_point_distance(ref, target)
        self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=alt_target))
        while self.vehicle.mode.name == "GUIDED":
            img = self.capture_camera()
            response = self.request_to_server(img)
            self.rect_and_pub_image(img, response.distance, response.tx, response.ty, response.bx, response.by)
            location = self.vehicle.location.global_relative_frame
            remainingDistance = self.get_point_distance((lat_target, lon_target), (location.lat, location.lon))
            print("Distance to target: %.3f, Heading %d"%(remainingDistance, self.vehicle.heading))
            if remainingDistance <= targetDistance*0.25: #Just below target, in case of undershoot.
                print("Reached target")
                break
            sleep(0.1)

    def start_mission(self):
        ref = self.vehicle.location.global_relative_frame
        self.goto((ref.lat, ref.lon, ref.alt), self.point)
        print("Go to target")
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

    def check_mission_status(self):
        location = self.vehicle.location.global_relative_frame
        distance = self.get_point_distance((location.lat, location.lon), self.point)
        print("Current Distance to target: %d"%distance)
        if abs(distance) < 0.5:
            self.on_mission = False
            self.attempt = 0
            self.num_of_detected += 1

    def rect_and_pub_image(self, img, distance, tx, ty, bx, by):
        img = cv2.rectangle(img, (tx, ty), (bx, by), (255, 255, 0), 4)
        img = cv2.putText(img, "%d"%distance, (bx, by), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_image.publish(img_msg)

    def capture_camera(self):
        image = self.camera_callback()
        return image

    def prepare_mission_first(self):
        if not self.on_mission:
            stat = self.prepare_mission()
            if stat:
                self.start_mission()
        else:
            image = self.capture_camera()
            response = self.request_to_server(image)
            self.rect_and_pub_image(image, response.distance, response.tx, response.ty, response.bx, response.by)
            self.check_mission_status()

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
        while alt > 0.2:
            alt = self.vehicle.location.global_relative_frame
            sleep(0.1)
        self.vehicle.armed = False

if __name__ == "__main__":
    try:
        drone = ARRGDrone()
#        drone.responses_loop()
    except KeyboardInterrupt:
        drone.vehicle.close()

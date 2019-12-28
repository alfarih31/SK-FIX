from time import sleep, time
from os import getenv
import math

from torch import set_flush_denormal
from dotenv import load_dotenv
from sensor_msgs.msg import CompressedImage
from turbojpeg import TurboJPEG
from pymavlink import mavutil

import dronekit
import rospy
import cv2
import cv_bridge

import grpc

from mission_msg.msg import Mission

import LOCAL_SK_pb2
import LOCAL_SK_pb2_grpc

from vision.ssd.fpnnet_ssd import create_fpn_ssd_predictor, create_fpnnet_ssd

load_dotenv()

set_flush_denormal(True)
class ARRGDrone:
    def __init__(self):
        self.class_names = [name.strip() for name in open(getenv("LABEL")).readlines()]
        num_classes = len(self.class_names)
        model = create_fpnnet_ssd(num_classes, is_test=True)
        model.load(getenv("MODEL"))
        self.predictor = create_fpn_ssd_predictor(model)

        self.turbo = TurboJPEG(lib_path=getenv("LIBTURBO"))
        rospy.init_node("NODE", anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.pub_image = rospy.Publisher("/image", CompressedImage, queue_size=1)
        self.pub_image_raw = rospy.Publisher("/image_raw", CompressedImage, queue_size=1)
        self.pub_mission = rospy.Publisher("/mission", Mission, queue_size=1)

        self.channel = grpc.insecure_channel("%s:%d"%(getenv("SERVER_ADDRESS"), int(getenv("SERVER_PORT"))))
        print("Channel started at %s"%getenv("SERVER_ADDRESS"))
        self.stub = LOCAL_SK_pb2_grpc.WSStub(self.channel)

        self.board = getenv("BOARD")
        self.baud = int(getenv("BAUDRATE"))
        print("BOARD: %s BAUD: %d"%(self.board, self.baud))
        self.vehicle = dronekit.connect(self.board, baud=self.baud, wait_ready=False)
        print("CONNECTED")
#        while not self.vehicle.is_armable:
#            print("WAITING INITIALIZING")
#            sleep(1)
        self.points = []
        self.distance = 0
        self.point = (0, 0, 0, 0, 0) # Lat, Lon, Alt, Head, Distance
        self.on_mission = False
        self.seq = 0
        self.attempt = 0
        self.num_of_detected = 0

        self.up_r = float(getenv("UP_RATIO"))
        self.down_r = float(getenv("DOWN_RATIO"))

        self.init_drone()

        self.responses_loop()

    def init_drone(self):
        self.vehicle.mode = dronekit.VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            self.vehicle.armed = True
            print("WAITING FOR ARMED")
            sleep(1)
        self.vehicle.simple_takeoff(alt=2)
        for _ in range(5):
            img = self.capture_camera()
            self.rect_and_pub_image(img)

        print("HOVER 2 Detik")
        self.set_attitude(duration=2)

        location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading
        while not heading:
            heading = self.vehicle.heading
            sleep(0.5)
        target = self.llg_from_distance(2, location.lat, location.lon, heading)
        self.goto((location.lat, location.lon), (*target, 0, 0, 0))
        print("INIT DRONE COMPLETE")

    def get_distance(self, h, w):
        if h == 0 or w == 0:
            return 0
        #distance = (self.k/(h-self.c))
        r = h/w
        if r >= self.down_r:
            distance = (27300/w)
        else:
            distance = (36400/w)
        return distance

    def predict(self, img):
        boxes, labels, _ = self.predictor.predict(img, 10, 0.6)
        if boxes.size(0) == 0:
            return False

        for i in range(boxes.size(0)):
            box = boxes[i, :]
            label = self.class_names[labels[i]]
            if label != 'person':
                box = False
                continue
            break
        return box

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


    def condition_yaw(self, heading, relative=False):
        if relative:
            is_relative = 1 #yaw relative to direction of travel
        else:
            is_relative = 0 #yaw is an absolute angle
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        self.vehicle.send_mavlink(msg)

    def maintain_yaw(self):
        print("Maintain Yawing")
        heading = self.vehicle.heading
        while not heading:
            heading = self.vehicle.heading
            sleep(0.1)

        target_heading = heading + 45
        if target_heading > 360:
            target_heading -= 360
        self.condition_yaw(target_heading)
        print("Current heading: %d, target: %d"%(heading, target_heading))
        for _ in range(10):
            heading = self.vehicle.heading
            print("Current heading: %d, target: %d"%(heading, target_heading))
            img = self.capture_camera()
            self.rect_and_pub_image(img)
            sleep(0.1)
        self.attempt += 1

    def prepare_mission(self):
        if self.attempt >= 7:
            print("Quad move forward")
            heading = self.vehicle.heading
            while not heading:
                heading = self.vehicle.heading
                sleep(0.1)
            location = self.vehicle.location.global_relative_frame
            lat, lon = self.llg_from_distance(2, location.lat, location.lon, self.vehicle.heading)
            self.goto((location.lat, location.lon, location.alt), (lat, lon, 2))
            self.attempt = 0

        img = self.capture_camera()
        self.distance = int(self.rect_and_pub_image(img, on_search=True)/100)
        for _ in range(2):
            if self.distance == 0:
                self.distance = int(self.set_attitude(duration=1, on_search=True)/100)
                if self.distance > 0:
                    break
                img = self.capture_camera()
                self.distance = int(self.rect_and_pub_image(img, on_search=True)/100)
            else:
                break

        location = self.vehicle.location.global_relative_frame
        heading = self.vehicle.heading
        if self.distance > 0:
            llg = self.llg_from_distance(self.distance+2, location.lat, location.lon, heading)
            status = self.check_point(llg)
            if status:
                self.point = (llg[0], llg[1], 2, heading, self.distance+2)
                return True
            else:
                self.maintain_yaw()
                return False
        else:
            self.maintain_yaw()
            return False

    def goto(self, ref, target):
        lat_target, lon_target, _, _, _ = target
        targetDistance = self.get_point_distance(ref, target)
        self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=2))
        while self.vehicle.mode.name == "GUIDED":
            img = self.capture_camera()
            self.rect_and_pub_image(img)
            location = self.vehicle.location.global_relative_frame
            remainingDistance = self.get_point_distance((lat_target, lon_target), (location.lat, location.lon))
            print("Distance to target: %.3f, Heading %d"%(remainingDistance, self.vehicle.heading))
            if remainingDistance <= targetDistance*0.25: #Just below target, in case of undershoot.
                print("Reached target No: %d"%self.num_of_detected)
                break
            elif remainingDistance >= targetDistance*0.3:
                self.vehicle.simple_goto(dronekit.LocationGlobalRelative(lat_target, lon_target, alt=2))
        self.set_attitude(duration=2)
            #sleep(0.01)

    def start_mission(self):
        print("Go to target No: %d"%self.num_of_detected)
        ref = self.vehicle.location.global_relative_frame
        self.goto((ref.lat, ref.lon, ref.alt), self.point)
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
        print("CHECKPOINT %d"%distance)
        if abs(distance) >= 2:
            return True
        else:
            return False

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
                             yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                             thrust=0.5):
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            while not yaw_angle:
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
                     thrust=0.5, duration=0, on_search=False):
        self.send_attitude_target(roll_angle, pitch_angle,
                                  yaw_angle, yaw_rate, False,
                                  thrust)
        start = time()
        while time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                      yaw_angle, yaw_rate, False,
                                      thrust)
            img = self.capture_camera()
            distance = self.rect_and_pub_image(img, on_search=on_search)
            if on_search and distance:
                return distance
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                                  0, 0, True,
                                  thrust)
        return 0

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
        if abs(distance) < 1:
            self.on_mission = False
            self.attempt = 0
            self.num_of_detected += 1

    def rect_and_pub_image(self, img, on_search=False):
        img_msg_raw = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_image_raw.publish(img_msg_raw)

        if on_search:
            box = self.predict(img)
            if isinstance(box, bool):
                box = [0, 0, 0, 0]
            cv2.rectangle(img, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)
            distance = self.get_distance(box[3]-box[1], box[2]-box[0])
            img = cv2.putText(img, "%d"%distance, (box[0], box[3]), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
        else:
            distance = 0
        cv2.imshow("From Client", img)
        if cv2.waitKey(1) == 27:
            self.vehicle.close()
            exit()
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_image.publish(img_msg)
        return distance

    def capture_camera(self):
        request = LOCAL_SK_pb2.toClient(on_mission=False)
        response = self.stub.clientStream(request)
        img_bytes = response.image
        image = self.turbo.decode(img_bytes)
        return image

    def prepare_mission_first(self):
        print("Check for on_mission")
        if not self.on_mission:
            stat = self.prepare_mission()
            if stat:
                self.start_mission()
        else:
            image = self.capture_camera() # Take Image
            self.rect_and_pub_image(image) # Predict, ROI, and Publish
            self.check_mission_status()

    def responses_loop(self):
        print("START LOOP")
        while True:
            try:
                if self.num_of_detected >= 1:
                    break
                self.prepare_mission_first()
            except KeyboardInterrupt:
                self.channel.close()
                break
        self.vehicle.mode = dronekit.VehicleMode("LAND")
        for _ in range(3):
            img = self.capture_camera()
            self.rect_and_pub_image(img)
            self.vehicle.mode = dronekit.VehicleMode("LAND")
        alt = self.vehicle.location.global_relative_frame.alt
        img = self.capture_camera()
        self.rect_and_pub_image(img)
        while alt > 0.2:
            img = self.capture_camera()
            self.rect_and_pub_image(img)
            alt = self.vehicle.location.global_relative_frame.alt
            sleep(0.1)
        self.vehicle.armed = False
        self.vehicle.close()

if __name__ == "__main__":
    try:
        drone = ARRGDrone()
#        drone.responses_loop()
    except KeyboardInterrupt:
        drone.vehicle.close()

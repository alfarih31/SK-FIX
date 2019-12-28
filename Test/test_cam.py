from turbojpeg import TurboJPEG
from dotenv import load_dotenv
from os import getenv

import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import CompressedImage

import grpc
import SK_pb2
import SK_pb2_grpc

load_dotenv()
bridge = cv_bridge.CvBridge()
turbo = TurboJPEG(lib_path="/usr/lib/x86_64-linux-gnu/libturbojpeg.so")

cap = cv2.VideoCapture()
cap.open(0)
while not cap.isOpened():
    pass

def start_stream(publisher):
    def rect_image(img, distance, tx, ty, bx, by):
        img = cv2.rectangle(img, (tx, ty), (bx, by), (255, 255, 0), 4)
        return cv2.putText(img, "%d"%distance, (bx, by), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 255, 0), 1)

    while True:
        ret, img = cap.read()
        if not ret:
            continue
        img_bytes = turbo.encode(img)
        request = SK_pb2.fromClient(image=img_bytes, on_mission=False)
        response = stub.clientStream(request)
        print(response.distance)
        img = rect_image(img, response.distance, response.tx, response.ty, response.bx, response.by)
        img_msg = bridge.cv2_to_compressed_imgmsg(img)
        publisher.publish(img_msg)


channel = grpc.insecure_channel("%s:%d"%(getenv("SERVER_ADDRESS"), int(getenv("SERVER_PORT"))))
stub = SK_pb2_grpc.WSStub(channel)
if __name__ == "__main__":
    rospy.init_node("CAM_NODE")
    publisher = rospy.Publisher("/image", CompressedImage, queue_size=1)
    start_stream(publisher)

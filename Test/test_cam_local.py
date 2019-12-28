from turbojpeg import TurboJPEG
from dotenv import load_dotenv
from os import getenv

from sensor_msgs.msg import CompressedImage

import cv2
import grpc
import LOCAL_SK_pb2
import LOCAL_SK_pb2_grpc

load_dotenv()
turbo = TurboJPEG(lib_path="/usr/lib/x86_64-linux-gnu/libturbojpeg.so")


def start_stream():

    while True:
        request = LOCAL_SK_pb2.toClient(on_mission=False)
        response = stub.clientStream(request)
        image = turbo.decode(response.image)
        cv2.imshow("TEST", image)
        if cv2.waitKey(27) == 27:
            break


channel = grpc.insecure_channel("%s:%d"%(getenv("SERVER_ADDRESS"), int(getenv("SERVER_PORT"))))
stub = LOCAL_SK_pb2_grpc.WSStub(channel)
if __name__ == "__main__":
    start_stream()

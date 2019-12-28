from concurrent import futures
from time import sleep
from turbojpeg import TurboJPEG

import cv2

import grpc
import LOCAL_SK_pb2
import LOCAL_SK_pb2_grpc

server_port = 50051

class LocalServicer(LOCAL_SK_pb2_grpc.WSServicer):
    def __init__(self):
        self.turbo = TurboJPEG(lib_path="/usr/lib/libturbojpeg.so")

        self.cap = cv2.VideoCapture()
        self.cap.open(0)
        while not self.cap.isOpened():
            pass
        print("CAM OPENED")

    def clientStream(self, request, context):
        ret, img = self.cap.read()
        while not ret:
            ret, img = self.cap.read()
            cv2.waitKey(1)
        img_bytes = self.turbo.encode(img, quality=90)
        response = LOCAL_SK_pb2.fromClient(image=img_bytes)
        return response

if __name__ == "__main__":
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    LOCAL_SK_pb2_grpc.add_WSServicer_to_server(LocalServicer(), server)
    server.add_insecure_port("[::]:%d"%server_port)
    server.start()
    print("Server Started On %d"%server_port)
    while True:
        try:
            sleep(86400)
        except KeyboardInterrupt:
            server.stop(0)
            exit()

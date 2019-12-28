from time import sleep, time
from os import getenv
from concurrent import futures
import cv2
import grpc
from torch import set_flush_denormal
from dotenv import load_dotenv
from turbojpeg import TurboJPEG, TJPF_BGR

import SK_pb2
import SK_pb2_grpc

from vision.ssd.mobilenet_v2_ssd_lite import create_mobilenetv2_ssd_lite, create_mobilenetv2_ssd_lite_predictor

load_dotenv()
server_port = 50051

set_flush_denormal(True)
class GCServicer(SK_pb2_grpc.WSServicer):
    def __init__(self):
        self.class_names = [name.strip() for name in open(getenv("LABEL")).readlines()]
        num_classes = len(self.class_names)
        model = create_mobilenetv2_ssd_lite(num_classes, is_test=True)
        model.load(getenv("MODEL"))
        self.predictor = create_mobilenetv2_ssd_lite_predictor(model)

        self.turbo = TurboJPEG(lib_path=getenv("LIBTURBO"))

        self.which_k = int(getenv("USE_K"))
        if self.which_k == 1:
            self.k = float(getenv("KH"))
            self.c = float()
        else:
            self.k = float(getenv("KW"))
        self.up_r = float(getenv("UP_RATIO"))
        self.down_r = float(getenv("DOWN_RATIO"))


    def get_distance(self, h, w):
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
                continue
            # w = box[2]-box[0]
            # h = box[3]-box[1]
            # r = h/w
            # if r > self.up_r or r < self.down_r:
            #     box = False
            #     continue
            break
        return box

    def clientStream(self, request, context):
        last_time = time()
        distance = 0
        image_buf = request.image
        image = self.turbo.decode(image_buf, pixel_format=TJPF_BGR)
        dt = time()-last_time
        last_time = time()
        print("FPS: %.2f"%(1/dt))
        if not request.on_mission:
            box = self.predict(image)
            if not isinstance(box, bool):
                cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (255, 255, 0), 4)
                distance = self.get_distance(box[3]-box[1], box[2]-box[0])
            else:
                box = [0, 0, 0, 0]
        cv2.imshow("From Client", image)
        if cv2.waitKey(max(int(32-(dt*1000)), 1)) == 27:
            exit()
        response = SK_pb2.toClient(distance=int(distance), tx=int(box[0]), ty=int(box[1]), bx=int(box[2]), by=int(box[3]))
        return response

if __name__ == "__main__":
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    SK_pb2_grpc.add_WSServicer_to_server(GCServicer(), server)
    server.add_insecure_port("[::]:%d"%server_port)
    server.start()
    print("Server Started On %d"%server_port)
    try:
        while True:
            sleep(86400)
    except KeyboardInterrupt:
        server.stop(0)
        exit()

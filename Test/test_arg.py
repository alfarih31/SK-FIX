import cv2
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--input1", required=True)
args = parser.parse_args()

print("FIle yg dibuka:", args.input1)
img = cv2.imread(args.input1)
cv2.imshow("TEST", img)
cv2.waitKey()

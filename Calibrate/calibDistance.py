from json import load
import cv2
import numpy as np

cam = cv2.VideoCapture(0)
cam.open(0)
while not cam.isOpened():
    pass

def main():
    with open("calibData.json", "r") as f:
        calibData = load(f)
    distCoef = np.array(calibData["distCoef"])
    cameraMat = np.array(calibData["cameraMat"])
    ret, image = cam.read()
    while not ret:
        ret, image = cam.read()
    h, w = image.shape[:2]
    print(h, w)
    outfile = open("to_calib.txt", "w")
    while True:
        ret, image = cam.read()
        if not ret:
            continue
        image = cv2.undistort(image, cameraMat, distCoef, None, cameraMat)
        rect = cv2.selectROI(image)
        if rect[0] == 0:
            continue
        print(rect)
        dist = int(input("Distance?\n"))
        for r in rect:
            outfile.writelines("%d,"%r)
        outfile.writelines("%d\n"%dist)
        print(rect)
        #print('w %d h %d cnt %s'%(w, h, str(c)))
        key = cv2.waitKey()
        if key == 27:
            break
    outfile.close()

if __name__ == '__main__':
    main()

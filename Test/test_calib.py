from json import load
import cv2
import numpy as np
with open("calibData.json", "r") as f:
    calibData = load(f)
distCoef = np.array(calibData["distCoef"])
cameraMat = np.array(calibData["cameraMat"])

image = cv2.imread("TEST.jpg")
h, w, _ = image.shape
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(cameraMat, distCoef,(w,h),1,(w,h))
x, y, w, h = roi
new_image = cv2.undistort(image, cameraMat, distCoef, None, newcameramtx)
new_image = new_image1[y:y+h, x:x+w]
cv2.imshow("TEST", new_image)
cv2.waitKey()
cv2.imwrite("TEST_UNDISTORT.jpg", new_image)
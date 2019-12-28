import cv2

cap = cv2.VideoCapture()
cap.open(0)
while not cap.isOpened():
    pass

ret, img = cap.read()
while not ret:
    ret, img = cap.read()
cv2.imshow("TEST", img)
cv2.waitKey()
cv2.imwrite("TEST.jpg", img)
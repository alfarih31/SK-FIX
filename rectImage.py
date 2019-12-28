import cv2

img = cv2.imread("/home/alfarihfz/Downloads/asa.jpeg")
print(img)
roi = cv2.selectROI(img)
x, y, w, h = roi
img = cv2.rectangle(img, (x, y), (x+w, y+h), (255, 255, 0), 2, 1)
cv2.imshow("IMG", img)
cv2.imwrite("/home/alfarihfz/Downloads/asa.jpeg", img)
cv2.waitKey()

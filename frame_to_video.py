import cv2
import glob

def sort_key(title):
    return int(title.split('/')[-1].split(".")[0].split("_")[-1])
list_image = glob.glob("/home/alfarihfz/Projects/gradient-ssd/predicted/*.jpg")
#list_image.extend(glob.glob("/home/alfarihfz/Projects/cb-skripsi/ISSIMM/ASA/images/*.jpg"))
list_image.sort(key=sort_key)
img = cv2.imread(list_image[0], cv2.IMREAD_COLOR)
h, w, _ = img.shape
out = cv2.VideoWriter('farih2.mp4',cv2.VideoWriter_fourcc(*"MJPG"), 30, (w, h))
#out.write(img)
for image in list_image[10:]:
    print(image)
    img = cv2.imread(image, cv2.IMREAD_COLOR)
    img = cv2.resize(img, (w, h))
    print(img.shape)
    cv2.imshow("TEST", img)
    out.write(img)
out.release()

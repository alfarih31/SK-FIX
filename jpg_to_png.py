import cv2
import glob

list_image = glob.glob("/home/alfarihfz/Projects/cb-skripsi/ISSIMM/ASA/images/*.png")

for image in list_image:
    img = cv2.imread(image)
    title = image.split(".png")[0].strip()
    cv2.imwrite(title+".jpg", img)

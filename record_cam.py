
from sensor_msgs.msg import CompressedImage
import cv_bridge
import rospy
import cv2

bridge = cv_bridge.CvBridge()

cap = cv2.VideoCapture()
cap.open(0)
while not cap.isOpened():
    pass

def start_stream(publisher: rospy.Publisher):
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, img = cap.read()
        if not ret:
            rate.sleep()
            continue
        img_msg = bridge.cv2_to_compressed_imgmsg(img)
        publisher.publish(img_msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("camera_node")
    pub_image = rospy.Publisher("/image", CompressedImage, queue_size=1)
    start_stream(pub_image)

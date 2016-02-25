import cv2
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def display_image(image):
	cv2.namedWindow('test', cv2.WINDOW_NORMAL)
	cv2.imshow('test', image)
	cv2.waitKey(0)

def main(args):
	path = args
	image = cv2.imread(path[1])
	#display_image(image)
	rospy.init_node('image_publisher', anonymous=True)
	image_pub = rospy.Publisher("image_pub", Image)
	try:
		while 1:
			image_pub.publish(CvBridge().cv2_to_imgmsg(image,"bgr8"))
	except KeyboardInterrupt:
		pass

if __name__ == "__main__":
	main(sys.argv)
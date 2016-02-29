import cv2
import sys
import rospy
import numpy as np
import tf
import cube_detector 
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_matrix

def display_image(image):
	cv2.namedWindow('test', cv2.WINDOW_NORMAL)
	cv2.imshow('test', image)
	cv2.waitKey(1)
	
class tag_tester:
	def __init__(self,
				 image_topic = '/kinect2/hd/image_color'):
		self.bridge = CvBridge()
		self.detector = cube_detector.cube_detector(None, 585)
		self.image_sub =  rospy.Subscriber(image_topic, Image, self.image_callback)

	def image_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		camera_matrix = self.detector.detect()
		if camera_matrix.size > 1:
			point = np.array([[0.05],[0.0],[0.0],[1.0]])
			print camera_matrix
			pix_point = np.dot(camera_matrix, point)
			print pix_point
			cv2.circle(cv_image, (int(pix_point[0,0] / pix_point[2,0]), int(pix_point[1,0] / pix_point[2,0])), 10, (0,0,155))
		
		display_image(cv_image)

def main():
	rospy.init_node('image_sub', anonymous=True)
	tag = tag_tester()
	rospy.spin()

if __name__ == "__main__":
	main()

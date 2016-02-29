import cv2
import sys
import rospy
import numpy as np
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker
from tf.transformations import quaternion_matrix

class cube_detector(object):
	def __init__(self, 
				 env,
				 marker_number, 
				 marker_topic = '/apriltags/marker_array', 
				 camera_topic = '/kinect2/hd/camera_info',
				 rgb_topic = '/kinect2/hd/image_color'):
		# try: 
		# 	rospy.init_node('cube_detector', anonymous=True)
		# except rospy.exception.ROSException:
		# 	pass
		self.env = env
		self.marker_number = marker_number
		self.marker_topic = marker_topic
		self.camera_topic = camera_topic
		self.rgb_topic = rgb_topic

	def detect(self, timeout=10):
		marker_message = rospy.wait_for_message(self.marker_topic, MarkerArray, timeout=timeout)
		camera_message = rospy.wait_for_message(self.camera_topic, CameraInfo, timeout=timeout)
		camera_matrix = np.array([0])
		for marker in marker_message.markers:
			if marker.id == self.marker_number:
				marker_pose = np.array(quaternion_matrix([
					marker.pose.orientation.x,
					marker.pose.orientation.y,
					marker.pose.orientation.z,
					marker.pose.orientation.w]))
				marker_pose[0,3] = marker.pose.position.x
				marker_pose[1,3] = marker.pose.position.y
				marker_pose[2,3] = marker.pose.position.z
				marker_pose = np.delete(marker_pose, 3, 0)
				
				camera_intrinsics = np.array(camera_message.K).reshape(3,3)

				print camera_intrinsics
				print marker_pose
				camera_matrix = np.dot(camera_intrinsics, marker_pose)
				print camera_matrix		
		return camera_matrix

	def extract_face_colors(self, tiemout=10):
		camera_matrix = self.detect()
		image = rospy.wait_for_message(self.rgb_topic, Image, timeout=timeout)
		#extrac the 8 faces not including the center face
		sd = 0.06
		for i in range(-1,2):
			for j in range(-1,2):
				pt = np.array([[i*sd],[j*sd],[0],[1]])
				pix_pt = np.dot(camera_matrx, pt)
				color = sample_pix()
# TesterMain

def main():
	detector = cube_detector(None, 585)
	while(1):
		detector.detect()

if __name__ == "__main__":
	main()

import cv2
import numpy as np
import math
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

W = 0
R = 1
Y = 2
O = 3
B = 4
G = 5

#Cube Model
class rcube:
	def __init__(self):
		self.data = np.zeros((6, 9))	

#What is this
def getEccentricity(mu):
	bigSqrt = math.sqrt(math.pow(mu['m20']-mu['m02'], 2) + 4 * math.pow(mu['m11'],2));
	if(bigSqrt < 0.001):
		return 99999
	return (mu['m20'] + mu['m02'] + bigSqrt) / (mu['m20']+mu['m02']-bigSqrt)

def extract_faces(image):	
	image_blur = cv2.medianBlur(image, 5)
	display_image(image_blur)

	#Run through a high pass filter first
	kernel = np.array([[0,1,0],[1,-4,1],[0,1,0]])
	image_hp = cv2.filter2D(image_blur, -1, kernel) * 10
	display_image(image_hp)
	
	#Convert it to grayscale
	image_gray = cv2.cvtColor(image_hp, cv2.COLOR_BGR2GRAY) 
	display_image(image_gray)
	ret, image_thresh = cv2.threshold(image_gray,30,200,cv2.THRESH_BINARY)

	opening_kernel = np.ones((1,1), np.uint8)
	image_thresh = cv2.morphologyEx(image_thresh, cv2.MORPH_OPEN, opening_kernel)
	display_image(image_thresh)

	#iso_kernel = np.array([[0,1,0],[1,0,1],[0,1,0]], np.uint8)
	#image_thresh = cv2.filter2D(image_thresh, -1, iso_kernel)
	#display_image(image_thresh)

	contours, hierarchy = cv2.findContours(image_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	con_list = []
	print "number of contours: %d" % len(contours)
	for cnt in contours:
		moment = cv2.moments(cnt)
		ecc = getEccentricity(moment)

		approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
		#print "ecc value %d" % ecc
		# print "number of approx_point %d" % len(approx)
		if(cv2.contourArea(cnt) > 50):
		#if(ecc > 10000):
			con_list.append(cnt)

	return con_list


def display_image(image):
	cv2.namedWindow('test', cv2.WINDOW_NORMAL)
	cv2.imshow('test', image)
	cv2.waitKey(0)

#apriltag callback
#def callback(self, data):

def process(image):
	image_resized = cv2.resize(image, None, fx=1, fy=1)
	con_list = extract_faces(image_resized)
	print "number of square contours: %d" % len(con_list)
	cv2.drawContours(image_resized, con_list, -1, (0,255,0), 1)
	cv2.namedWindow('cube', cv2.WINDOW_NORMAL)
	cv2.imshow('cube', image_resized)
	cv2.waitKey(0)

def main(args):
	image = cv2.imread('../data/data1.jpg')
	rospy.init_node('cube_perception', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting donw")

if __name__ == "__main__":
	main(sys.argv)
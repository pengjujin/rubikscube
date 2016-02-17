import cv2
import numpy as np
import math

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
	lab_iamge = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
	image_blur = cv2.blur(image, (20,20))
	display_image(image_blur)

	#Run through a high pass filter first
	kernel = np.array([[0,1,0],[1,-4,1],[0,1,0]])
	image_hp = cv2.filter2D(image_blur, -1, kernel)
	image_gray = cv2.cvtColor(image_hp, cv2.COLOR_BGR2GRAY) * 10
	ret, image_thresh = cv2.threshold(image_gray,20,200,cv2.THRESH_BINARY)
	#dial_kernel = np.ones((2,2), np.uint8)
	#edges = cv2.Canny(image, 300, 350)
	#edges = cv2.dilate(edges, dial_kernel, iterations = 2)
	display_image(image_thresh)
	contours, hierarchy = cv2.findContours(image_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	con_list = []
	print "number of contours: %d" % len(contours)
	for cnt in contours:
		moment = cv2.moments(cnt)
		ecc = getEccentricity(moment)

		approx = cv2.approxPolyDP(cnt, 0.08*cv2.arcLength(cnt, True), True)
		#print "ecc value %d" % ecc
		# print "number of approx_point %d" % len(approx)
		if(len(approx) == 4 ):
		#if(ecc > 10000):
			con_list.append(cnt)
		
	return con_list

def display_image(image):
	cv2.namedWindow('test', cv2.WINDOW_NORMAL)
	cv2.imshow('test', image)
	cv2.waitKey(0)

def test():
	image = cv2.imread('../data/mix_cube.JPG')
	con_list = extract_faces(image)
	print "number of square contours: %d" % len(con_list)
	cv2.drawContours(image, con_list, -1, (0,255,0), 1)
	cv2.namedWindow('cube', cv2.WINDOW_NORMAL)
	cv2.imshow('cube', image)
	cv2.waitKey(0)

if __name__ == "__main__":
	test()
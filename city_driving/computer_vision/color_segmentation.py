import cv2
import numpy as np
import pdb



#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	#ORANGE_THRESHOLD = ([5,50,50], [15,255,255])
	# 
	# 
	# 
	#   #HSV (Hue, Saturation, Value)   For Orange color
	ORANGE_THRESHOLD = ([5,80,100], [18,255,255])
	bounding_box = ((0,0),(0,0))
	

	frame = img
	# Taking a matrix of size 5 as the kernel
	kernel = np.ones((5,5), np.uint8)

	# The first parameter is the original image,
	# kernel is the matrix with which image is
	# convolved and third parameter is the number
	# of iterations, which will determine how much
	# you want to erode/dilate a given image.
	img_erosion = cv2.erode(frame, kernel, iterations=1)
	img_dilation = cv2.dilate(img_erosion, kernel, iterations=2)
	# image_print(frame)
    # image_print(img_erosion)
	# image_print(img_dilation)

	frame = img_dilation
	
	# image_print([])
	# Taking a matrix of size 5 as the kernel


	#Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)



	#OpenCV needs bounds as numpy arrays
	lower_bound = np.array(ORANGE_THRESHOLD[0])
	upper_bound = np.array(ORANGE_THRESHOLD[1])



	mask = cv2.inRange(hsv,lower_bound,upper_bound)
	# image_print(mask)
	bounding_box = ((0,0),(0,0))




	_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if len(contours)!=0:
		c = max(contours, key = cv2.contourArea)
		x,y,w,h = cv2.boundingRect(c)
		bounding_box = ((x,y),(x+w,y+h))



	

	########### YOUR CODE ENDS HERE ###########

	return bounding_box

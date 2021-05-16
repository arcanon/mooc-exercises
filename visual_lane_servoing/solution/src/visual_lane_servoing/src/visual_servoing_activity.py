#!/usr/bin/env python
# coding: utf-8

# In[1]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    global initialized
    global veh
    import rospy

    # good norm left_factor = 0.20
    left_factor = 0.002

    try:
        left_factor = float(rospy.get_param('/left_factor'))
        print(left_factor)
    except:
        print("error with left_factor")

    
    steer_matrix_left_lane = -1*np.ones(shape)*left_factor

    return steer_matrix_left_lane


# In[120]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """

    # good default right_factor = 0.4
    right_factor = 0.004
    import rospy

    try:
        right_factor = rospy.get_param('/right_factor')
        print(right_factor)
    except:
        print("error with right_factor")
    
    steer_matrix_right_lane = np.ones(shape)*right_factor

    return steer_matrix_right_lane


# In[32]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape

    
    #image = cv2.imread('../images/visual_control/pic1_rect.png')

    
    
    
    #mgbgr = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2RGB)
    
    #imgrgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
   
    mask_ground = np.zeros(img.shape, dtype=np.uint8) # TODO: CHANGE ME
    print(img.shape)

    import rospy
    mask_factor = 0.82
    try:
        mask_factor = float(rospy.get_param('/mask_factor'))
        print(mask_factor)
    except:
        print("error with mask_factor")

    mid = mask_factor*img.shape[0]
    mask_ground[img.shape[0]-int(mid):img.shape[0],:] = 1
    
    #Real
    #white_lower_hsv = np.array([0, 0, 190])         # CHANGE ME
    #white_upper_hsv = np.array([180, 38, 255])   # CHANGE ME
    #yellow_lower_hsv = np.array([21, 120, 64])        # CHANGE ME
    #yellow_upper_hsv = np.array([40, 255, 255])  # CHANGE ME
    #sim
    white_lower_hsv = np.array([0, 0, 170])         # CHANGE ME
    white_upper_hsv = np.array([180, 38, 255])   # CHANGE ME
    yellow_lower_hsv = np.array([21, 80, 64])        # CHANGE ME
    yellow_upper_hsv = np.array([40, 255, 255])  # CHANGE ME

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)
    
    sigma = 5 # CHANGE MEmask_left

    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)
    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(width/2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(width/2))] = 0
    #print(mask_left.shape)
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    
    sobelx = cv2.Sobel(img,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img,cv2.CV_64F,0,1)
    
    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)

    # Compute the orientation of the gradients
    Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)
    
    threshold = 28 # CHANGE ME

    mask_mag = (Gmag > threshold)
    
    mask_left_edge = mask_ground * mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    #mask_left_edge = mask_yellow
    mask_right_edge = mask_ground * mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    #mask_right_edge = mask_white
    
    return (Gmag* mask_left_edge, Gmag * mask_right_edge)


#!/usr/bin/python
###############################################################################
# 
# blocks.py
#
#   Python OpenCV program to identify blocks on a white page,
#   determine their orientation angle and x,y position coordinate, and their color.
#   This is a study for a Python program to be incorporated into my robotic arm
#   project, to identify the blocks and then pick and place them in a sorted pile.
#
#   Resources:
#       Thresholding: https://docs.opencv.org/3.2.0/d7/d4d/tutorial_py_thresholding.html
#       Contour detection: http://answers.opencv.org/question/179004/how-to-detect-object-and-pattern-and-retrieve-the-angle-of-orientation/
#       Detection in ROI: https://stackoverflow.com/questions/42004652/how-can-i-find-contours-inside-roi-using-opencv-and-python
#       Erode and Dilate: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
#       Object color: https://www.pyimagesearch.com/2016/02/15/determining-object-color-with-opencv/
#
#   June 28, 2018
#
###############################################################################

import numpy as np
import cv2
import opencvconst as cv
import math
import colorlabeler

#
# Globals
#
MEDIA_DIR = './media'
PICTURE_FILE = '/blocks5.jpg'
IMAGE = MEDIA_DIR + PICTURE_FILE
VIEW_WIDTH = 640

#ROI_XYHW = (200,300,1500,2500)     # TODO Pick the (x,y,h,w) ROI with mouse on the image
#ROI_XY   = (200,300,2700,1800)     #      In the form of (x0,y0,x1,y1)

ROI_XY   = (10,20,620,450)          # In the form of (x0,y0,x1,y1)
ORIGIN_PIX = (397,458)              # Robot arm grid coordinate origin
PIX_PER_MM = 2.24                   # Robot arm grid coordinate resolution in pixels per mm

#
# Kernel for dilation or erosion
#
dilate_kernel = np.ones((5, 5), np.uint8)
erode_kernel = np.ones((5, 5), np.uint8)

def main():
    """Identify blocks on a white page, their position, orientation, and color."""
    
    # Initialize display window and font
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.namedWindow('blocks', cv2.WINDOW_NORMAL)
    
    # Initialize color detector class
    cl = colorlabeler.ColorLabeler()
    
    # Read image
    orig_img = cv2.imread(IMAGE)
   
    # Scale image to fit a smaller window
    window_scale = float(orig_img.shape[0]) / float(orig_img.shape[1])
    window_width = VIEW_WIDTH
    window_height = int(window_width * window_scale)
    #img = cv2.resize(orig_img, (window_width, window_height))
    img = orig_img
    
    # Process the image's gray scale to "clean up" noise
    blur = cv2.GaussianBlur(img, (5,5), 0)
    img_Lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
    gray_image = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(gray_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    mask_erode = cv2.erode(thresh, erode_kernel, iterations = 1)
    mask = cv2.dilate(mask_erode, dilate_kernel, iterations = 3)
    #mask = cv2.bitwise_not(mask)

    # Detect contours
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    i = 0
    for cnt in contours:
        rect = cv2.minAreaRect(cnt)     # rect is a Box2D: center (x,y), (width, height), angle of rotation of the box
        box = np.asarray(cv2.boxPoints(rect),dtype=np.int0)
        contour_area = int(rect[1][0] * rect[1][1])
        
        b = box.transpose()

        # Filter contours within the ROI
        if (min(b[0]) >= ROI_XY[0] and min(b[0]) <= ROI_XY[2]  and \
            max(b[0]) >= ROI_XY[0] and max(b[0]) <= ROI_XY[2]) and \
           (min(b[1]) >= ROI_XY[1] and min(b[1]) <= ROI_XY[3]  and \
            max(b[1]) >= ROI_XY[1] and max(b[1]) <= ROI_XY[3]) and \
           (contour_area >= 1500 and contour_area <= 2500):
            cv2.drawContours(img,[box],0,(0,255,0),1)
            contour_rotation = get_rotation(box)
            box_color = color = cl.label(img_Lab, cnt)
            i = i + 1
            contour_text = 'contour {}'.format(i)
            px, py = int(rect[0][0]), int(rect[0][1])
            x, y = (ORIGIN_PIX[0] - px) / PIX_PER_MM, (ORIGIN_PIX[1] - py) / PIX_PER_MM
            cv2.putText(img, contour_text, (px,py), font, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.drawMarker(img, ORIGIN_PIX, (0,255,0), thickness = 2)
            print 'Contour #{}: ({},{}) pix, ({:.2f},{:.2f}) [mm], {:.2f} [deg], area {}, {}'.format(i, px, py, x, y, contour_rotation, contour_area, box_color)
    
    # Display results
    #cv2.imshow('thresh', thresh)
    #cv2.imshow('erode', mask_erode)
    #cv2.imshow('dilate', mask)
    #cv2.imshow('Lab', img_Lab)
    cv2.imshow('blocks', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

###############################################################################
#
# get_rotation()
#
def get_rotation(box):
    """Return the rotation of a rectangle contour based on the slope of one of its long side."""
    
    # Take the first three vertices and calculate side lengths of the right angle triangle they form
    d1 = math.sqrt(pow((box[0][0]-box[1][0]),2)+pow((box[0][1]-box[1][1]),2))
    d2 = math.sqrt(pow((box[1][0]-box[2][0]),2)+pow((box[1][1]-box[2][1]),2))
    d3 = math.sqrt(pow((box[2][0]-box[0][0]),2)+pow((box[2][1]-box[0][1]),2))

    # Select vertices.
    # The shortest distance is the short side, the longest is the rectangle's diagonal.
    if (d1 > d2 and d1 < d3) or (d1 > d3 and d1 < d2):
        x1 = box[0][0]
        y1 = box[0][1]
        x2 = box[1][0]
        y2 = box[1][1]
    
    elif (d2 > d1 and d2 < d3) or (d2 > d3 and d2 < d1):
        x1 = box[1][0]
        y1 = box[1][1]
        x2 = box[2][0]
        y2 = box[2][1]
    
    elif (d3 > d1 and d3 < d2) or (d3 > d2 and d3 < d1):
        x1 = box[2][0]
        y1 = box[2][1]
        x2 = box[0][0]
        y2 = box[0][1]
    
    # Calculate the line rotation
    if x1 == x2:
        rotation = 90.0
    else:
        rotation = 57.29578049 * math.atan(float(y2-y1) / float(x2-x1))
        
    return rotation

###############################################################################
#
# Startup
#
if __name__ == '__main__':
    main()


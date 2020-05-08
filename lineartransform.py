#!/usr/bin/python
###############################################################################
# 
# linertransform.py
#
#   Python OpenCV program to calculate a linear transform from webcam pixel
#   space to millimeter coordinate grid.
#   This code and algorithms will be incorporated in the robotic arm project in order
#   to reduce the time it takes to mechanically align camera to grid table.
#
#   August 25, 2018
#
###############################################################################

import numpy as np
import cv2
import math

import opencvconst as cv

def main():
    """calculate a linear transform from webcam pixel space to millimeter coordinate grid."""
    global x0, y0, click, move
    
    print 'hit <ESC> key to exit.'
    
    # Initialize display window and font
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.namedWindow('grid', cv2.WINDOW_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('grid', mouse_click)
    click = False
    move = False
    points = 0
    pixel_per_mm = 0.0
    px = 0  # Top-left point in pixel coordinates
    py = 0
    
    p = []
    
    # Top-left (p0) and bottom-right p(1) grid calibration point coordinates
    p0 = (250,50)   # Grid top left
    p1 = (150,-50)  # Grid bottom-right
    
    # Open the webcam device
    cap = cv2.VideoCapture(0)
    
    #
    # frame capture and processing loop
    #
    while(True):

        cap_ok, img = cap.read()
        if not cap_ok:
            break
            
        if click:
            print 'point:', x0, y0

            if points == 2:
                points = 0
                del p[:]
                pixel_per_mm = 0.0

            p.append((x0,y0))
            points = points + 1
            click = False

        # Two clicks were registered
        if points == 2:
            # Draw rectangle
            cv2.rectangle(img,p[0],p[1],(255,0,0))
            # Recalculate coordinate transform
            if pixel_per_mm == 0.0:
                avg_pixel_distance = (abs(p[0][0]-p[1][0]) + abs(p[0][1]-p[1][1]))/2.0
                pixel_per_mm = avg_pixel_distance / 100.0
                px = p[0][0]
                py = p[0][1]
                print pixel_per_mm,'[pix/mm]'
        
        if move and pixel_per_mm > 0.0:
            x, y = ((py - y0) / pixel_per_mm) + p0[0], ((px - x0) / pixel_per_mm) + p0[1]
            print x, y
            move = False
    
        #
        #   key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

        # Display results
        cv2.imshow('grid', img)
    
    cv2.destroyAllWindows()
    
###########################################################
#
# mouse_click()
#
#   Mouse event callback function.
#   Used to capture mouse event and mark object for tracking.
#
#   param:  event type, mouse coordinates and event parameters
#   return: nothing, marks a frame region for tracker initialization
#
def mouse_click(event, x, y, flags, param):
    """Mouse callback that returns x and y coordinates of pointer and an action flag."""
    global x0, y0, click, move
    
    if event == cv2.EVENT_LBUTTONDOWN:
        x0 = x
        y0 = y
        click = True
        
    elif event == cv2.EVENT_MBUTTONDOWN:
        x0 = x
        y0 = y
        move = True


###############################################################################
#
# Startup
#
if __name__ == '__main__':
    main()


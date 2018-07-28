###############################################################################
# 
# peoplecount.py
#
#   Python OpenCV program to count people entering and exiting a room.
#   Using sequential frame subtraction method.
#   Source: https://www.pyimagesearch.com/2015/05/25/basic-motion-detection-and-tracking-with-python-and-opencv/
#   Adapted to use live webcam feed.
#
#   This implementation uses a webcam pointed across the room entry way,
#   and counts people entering and exiting the room.
#   This implementation uses a simple frame subtraction and threshold method
#   to find differences between consecutive frames. The differences are
#   interpreted as motion. The delta image is thresholded and then searched for contours.
#   Because there may be many contours, the program finds the bounding box for
#   *all* contours, and then marks that 'super' bounding box on the video.
#   The 'super' contour is tracked vs. two vertical lines on the video frame,
#   and a pattern of crossing both lined in one direction is counted as entry or
#   exit from the room.
#   Because the camera is not mounted from above, counting might not be accurate.
#
#   February 11, 2018
#
###############################################################################

import time
import sys
import numpy as np
import cv2
import threading
import subprocess

import opencvconst as cv
import expfilter as ep

def main():
    global tts

    """
    Captures input frame from webcam.
    calculates and displays count of object going in or out of a room.
    """
    
    #
    # Initializations
    #
    trace_title = False
    trace = False
    ref_time = time.time()
    
    fps_text = '?? Fps'
    frame_count = 0
    start = time.time()

    sense_line1 = 240
    sense_line2 = 340
    threshold_hysteresis = 10
    
    room_occupancy = 0
    occupancy_text = 'Occupancy: 0'
    count = 0
    count_text = 'Count: 0'
    
    NO_CROSS = 0        # no crossing in progress
    CROSS_IN = 1        # object crossed first sense vector on way into room
    CROSS_OUT = 2       # object crossed second sense vector on way out of room
    CROSS_BLOCK = 4     # block counter to prevent spurious counting
    state = NO_CROSS
    inhibit_start = time.time()

    dilate_kernel = np.ones((7, 7), np.uint8)
    prev_x = 0
    min_y = 0
    max_y = 0
    min_x = 0
    max_x = 0
    
    cv2.namedWindow('frame', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    #cv2.namedWindow('delta', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    sign = lambda a: (a>0) - (a<0)
    
    #
    # Construct two exponential filter objects.
    #
    filer = ep.ExpFilter(alpha=0.1)
    
    #
    # Open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = int(cap.get(cv.CV_CAP_PROP_FRAME_WIDTH))
        frameHeight = int(cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT))
        
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)

    #
    # Take first frame and find corners in it
    #
    cap_ok, frame = cap.read()
    if not cap_ok:
        sys.exit()

    prev_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    prev_frame = cv2.GaussianBlur(prev_frame, (21, 21), 0)

    while(True):

        cap_ok, frame = cap.read()
        if not cap_ok:
            break
        
        curr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        curr_frame = cv2.GaussianBlur(curr_frame, (21, 21), 0)

        #
        # Subtract one frame from the other to detect changes
        #
        frame_delta = cv2.absdiff(prev_frame, curr_frame)

        #
        # Threshold and dilation
        #
        thresh = cv2.threshold(frame_delta, 10, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, dilate_kernel, iterations = 1)

        #
        # Find contours, but instead of drawing all of them
        # only figure out: the topmost or lowest X pixel coordinate,
        # and the left and right most Y pixel coordinate between all
        # detected contours.
        #
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found_contours = False
        coord_initialized = False
        
        for contour in contours:

            if not coord_initialized:               # initialize once
                min_y = 999
                max_y = -1
                min_x = 999
                max_x = -1
                coord_initialized = True
            
            found_contours = True
            (x, y, w, h) = cv2.boundingRect(contour)
            if y < min_y:
                min_y = y
            if (y+h) > max_y:
                max_y = (y+h)
            if x < min_x:
                min_x = x
            if (x+w) > max_x:
                max_x = x+w

        #
        # Filter and calculate displacement and direction
        # of movement
        #
        curr_x = int(filer.filter((min_x+max_x)/2))
        
        if found_contours:
            direction = sign(prev_x-curr_x)
        else:
            direction = 0
        
        #
        # State machine to track entries and exits
        # States:
        #   NO_CROSS    : no crossing in progress
        #   CROSS_IN    : object crossed first sense vector on way into room
        #   CROSS_OUT   : object crossed second sense vector on way out of room
        #   CROSS_BLOCK : block counter to prevent spurious counting
        #
        if curr_x > sense_line1 and curr_x < sense_line2 and direction < 0:
            state = CROSS_OUT

        if curr_x > sense_line1 and curr_x < sense_line2 and direction > 0:
            state = CROSS_IN

        if curr_x > sense_line2 and direction < 0 and room_occupancy > 0:
            if state == CROSS_OUT:
                room_occupancy -= 1
                if room_occupancy < 0:
                    room_occupancy = 0
                state = CROSS_BLOCK
                inhibit_start = time.time()
                threading.Thread(target=thread_announce, args=('media/goodby.wav',)).start()
                
        if curr_x < sense_line1 and direction > 0:
            if state == CROSS_IN:
                room_occupancy += 1
                count += 1
                state = CROSS_BLOCK
                inhibit_start = time.time()
                threading.Thread(target=thread_announce, args=('media/hello.wav',)).start()

        inhibit_time = time.time() - inhibit_start
        if state == CROSS_BLOCK and inhibit_time >= 1:
            state = NO_CROSS
        
        occupancy_text = 'Occupancy: {}'.format(room_occupancy)
        count_text = 'Count: {}'.format(count)
        
        #
        # Calculate and display FPS.
        # Use the 10sec interval to also poll the NXT for battery level.
        #
        frame_count = frame_count + 1
        end = time.time()
        measure_interval = end - start
        if measure_interval > 10:
            fps = frame_count / measure_interval
            fps_text = '{:.2f} Fps'.format(fps)
            frame_count = 0
            start = time.time()

        #
        # Add information to the video
        #
        cv2.putText(frame, '1', (sense_line1, 50), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.line(frame, (sense_line1, 55), (sense_line1, frameHeight), (0, 255, 0), 1)
        
        cv2.putText(frame, '2', (sense_line2, 50), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.line(frame, (sense_line2, 55), (sense_line2, frameHeight), (0, 255, 0), 1)
        
        cv2.putText(frame, fps_text, (1, 20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, occupancy_text, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, count_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
               
        if found_contours:
            #cv2.rectangle(frame, (min_x, min_y), (max_x, max_y), (0, 255, 0), 1)
            cv2.drawMarker(frame, (curr_x, frameHeight-20), (0, 255, 0), cv2.MARKER_TRIANGLE_UP)

            if direction > 0:
                cv2.putText(frame, 'IN', (1, 80), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
            elif direction < 0:
                cv2.putText(frame, 'OUT', (1, 80), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        else:
            cv2.drawMarker(frame, (curr_x, frameHeight-20), (0, 0, 255), cv2.MARKER_TRIANGLE_UP)

        #
        # Display result and check for escape or keyboard command key
        #
        cv2.imshow('frame',frame)
        #cv2.imshow('delta',thresh)
        
        k = cv2.waitKey(1) & 0xff
        # exit
        if k == 27:
            break
        # reset occupancy counter
        if k == ord('r'):
            room_occupancy = 0
            count = 0
            state = NO_CROSS

        #
        # Update the previous frame and previous points
        #
        prev_frame = curr_frame.copy()
        prev_x = curr_x

    cv2.destroyAllWindows()
    cap.release()

###########################################################
#
# list_avg()
#
#   Remove outliers from a list and return the
#   element average without the outliers
#
#   param:  list of integers
#   return: average in float
#
def list_avg(l):
    """Calculate average of an integer list after removing outliers. Return average as float"""
    
    l.sort()
    return sum(l[1:-1])/float(len(l[1:-1]))

###########################################################
#
# thread_announce()
#
#   Play and announcement WAV file passed in as a parameter.
#   TODO This function does not check for file existence
#        nor throw an exception if file is not found
#
#   param:  WAV sound file name
#   return: none
#
def thread_announce(announcement_wav_file):
    """Play and announcement WAV file passed in as a parameter"""
    
    subprocess.call(['aplay', announcement_wav_file])
    
#
# Startup
#
if __name__ == '__main__':
    main()

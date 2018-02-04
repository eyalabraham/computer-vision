###############################################################################
# 
# trackers.py
#
#   Python OpenCV program for testing different trackers.
#   It is based on code from [https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/]
#
#   The program has three operating modes:
#   'm' key Mark: enter this mode while holding an object in front of the webcam,
#       and use the mouse to select it with a rectangle.
#   's' key Show: this is a test mode that shows the tracked object. Use this mode
#       to test tracking on the display.
#   
#   TODO
#   't' key Track: this mode activates the motors and sends power controls to them.
#       The Track mode will be halted (red 'Track' symbol) if the blob is lost, or
#       more than one blob is detected. Tracking will resume automatically when
#       a single blob is re-detected.
#
#   January 28, 2018
#
###############################################################################

import sys
import numpy as np
import cv2
import opencvconst as cv
import time

###############################################################################
#
# main()
#
def main():
    """Control function that reads webcam, and tracks a marked object."""
    global x0, y0, x1, y1, drawing, mode, frame, bbox, tracker, tracker_initialized
    global MODE_MARK

    #
    # initialization
    #
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    
    MODE_TRACK = 0          # track an object
    MODE_SHOW = 1           # only show tracking markers on video
    MODE_MARK = 2           # mark region color to track
    MODE_TRACK_HOLD = 3     # temporarily suspend tracking until object is recaptured

    tracker_initialized = False
    bbox = (0, 0, 0, 0)
    last_good_bbox = bbox
    
    mode = MODE_SHOW
    mode_text = 'Show'
    fps_text = '?? Fps'
    
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
    
    print ' m - mark color region to track\n s - display tracking marker only\n ESC - quit'

    #
    # Initialize tart time and frame count
    #
    frame_count = 0
    start = time.time()

    #
    # link event callback function
    #
    cv2.namedWindow('image', cv2.WINDOW_GUI_NORMAL+cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('image', mark_rect)

    #
    # setup font for overlay text 
    #
    font = cv2.FONT_HERSHEY_SIMPLEX

    #
    # Set up tracker.
    # 
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
    tracker_type = tracker_types[2]
 
    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type)
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()

    #
    # open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        #cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        #cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)
        
        frameCenterX = int(frameWidth/2)
        frameCenterY = int(frameHeight/2)
    else:
        sys.exit()

    #
    # frame capture and processing loop
    #
    while(True):
        #
        # capture a frame
        # cover to appropriate color space to improve detection
        # in different lighting conditions
        #
        cap_ok, frame = cap.read()
        if not cap_ok:
            break

        #
        # Operations on the captured image done here.
        # If marking a section on the object color for tracking
        # then only display the selection
        #
        if mode == MODE_MARK:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 1)

        #
        # If tracking or only displaying object tracking information
        # then draw the tracking markers on the frame before it is displayed.
        # Only do this if the tracker was initialized
        #
        elif tracker_initialized:
            #
            # Update the tracker with the newly acquired frame.
            #
            track_ok, bbox = tracker.update(frame)

            if track_ok:
                last_good_bbox = bbox
                
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2)

                object_x = int(bbox[0] + bbox[2]/2)
                object_y = int(bbox[1] + bbox[3]/2)

                err_pan_i = frameCenterX - object_x
                err_tilt_i = frameCenterY - object_y

                cv2.arrowedLine(frame, (frameCenterX, frameCenterY), (object_x, object_y), (255, 0, 0), 2)

                if mode == MODE_TRACK_HOLD:
                    mode = MODE_TRACK
            else:
                p1 = (int(last_good_bbox[0]), int(last_good_bbox[1]))
                p2 = (int(last_good_bbox[0] + last_good_bbox[2]), int(last_good_bbox[1] + last_good_bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 0, 255), 1)

                if mode == MODE_TRACK:
                    mode = MODE_TRACK_HOLD

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
        # Add text and other markers to the image
        #
        if mode == MODE_TRACK_HOLD:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, tracker_type, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        
        #
        # Display the resulting frame
        #
        cv2.imshow('image', frame)
              
        #
        #   key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break
        elif key == ord('m'):
            x0,y0  = -1,-1
            x1,y1  = -1,-1
            mode_text = 'Mark'
            mode = MODE_MARK
        elif key == ord('s'):
            mode_text = 'Show'
            mode = MODE_SHOW
        else:
            pass

    #
    # When done, release the capture.
    #
    cap.release()
    cv2.destroyAllWindows()

###########################################################
#
# mark_rect()
#
#   Mouse event callback function.
#   Used to capture mouse event and mark object for tracking.
#
#   param:  event type, mouse coordinates and event parameters
#   return: nothing, marks a frame region for tracker initialization
#
def mark_rect(event, x, y, flags, param):
    """Mouse callback that marks a frame region for tracker initialization."""
    global x0, y0, x1, y1, drawing, mode, frame, bbox, tracker, tracker_initialized
    global MODE_MARK

    if mode != MODE_MARK:
        return
        
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        x0, y0 = x, y
        x1, y1 = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            x1, y1 = x, y

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

        #
        # Convert any start (x0,y0) and end (x1,y1) points to be
        # a top-left to bottom right pair.
        # Extract ROI and initialize the tracker
        #
        if x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0:
            return
        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0

        bbox = (x0, y0, x1-x0, y1-y0)
        tracker_initialized = tracker.init(frame, bbox)
        print 'tracker.init()', tracker_initialized

#
# Startup
#
if __name__ == '__main__':
    main()

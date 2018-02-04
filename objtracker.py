###############################################################################
# 
# objtracker.py
#
#   Python OpenCV and Lego NXT program for tracking object with a webcam
#   and a pan-tilt system.
#   It is based on code from [https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/]
#
#   The program requires a connected webcam and an NXT pan-tilt mechanical setup.
#   The program has three operating modes:
#   'm' key Mark: enter this mode while holding an object in front of the webcam,
#       and use the mouse to select it with a rectangle.
#   's' key Show: this is a test mode that shows the tracked object. Use this mode
#       to test tracking on the display.
#   't' key Track: this mode activates the motors and sends power controls to them.
#       The Track mode will be halted (red bounding box) if the object is lost
#       Tracking will resume automatically when the tracker algorithm re-detects
#       the object. With the default selected tracker algorithm set to KCF
#       simply bringing the object back into the the view of the red bounding box.
#
#   February 3, 2018
#
###############################################################################

import sys
import nxt
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
    batt_level = 0.0
    batt_level_text = '?? [v]'
    cvs_title_printed = False
    
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
    
    #
    # PID constants for pan and tilt PID controller.
    # These constants are tuned for the worm-gear mechanical setup
    #
    pan_P = 0.8
    pan_I = 0.05
    pan_D = 0.1
    
    tilt_P = 0.6
    tilt_I = 0.05
    tilt_D = 0.1

    print ' m - mark color region to track\n t - track\n s - display tracking marker only\n ESC - quit'

    #
    # connect to NXT device
    # define motor references
    #
    devNXT = nxt.locator.find_one_brick()
    tilt_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_A)
    pan_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_B)

    #
    # Initialize start time and frame count.
    # Initialize a reference start time for the CSV output trace
    #
    frame_count = 0
    start = time.time()
    ref_time = time.time()

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
    # Tracker algorithm is hard coded here to default tracker KCF.
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
    # Open the capture device and print some
    # useful properties.
    # This tracker program will leave the default webcam frame size
    # that is 640x480 for the webcam I am using.
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
        # If marking a section on the frame for tracking
        # then only display the ROI selection
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

            #
            # If the tracker update was successful, object still being tracked, then
            # update the prev bounding box position and proceed to:
            # - display the tracker bounding box
            # - an arrow line from frame center to the object center
            # - calculate pan and tilt error from frame center
            #
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
            
            #
            # If tracking is lost for some reason then use the last location
            # of the bounding box to mark that last location with a red box
            #
            else:
                p1 = (int(last_good_bbox[0]), int(last_good_bbox[1]))
                p2 = (int(last_good_bbox[0] + last_good_bbox[2]), int(last_good_bbox[1] + last_good_bbox[3]))
                cv2.rectangle(frame, p1, p2, (0, 0, 255), 1)

                if mode == MODE_TRACK:
                    mode = MODE_TRACK_HOLD

        #
        # Only when in tracking mode activate the motors,
        # and use PID calculations to control the pan-tilt device
        #
        if mode == MODE_TRACK and tracker_initialized:
            #
            # First apply an exponential filter to the tracker position error.
            # info: https://en.wikipedia.org/wiki/Exponential_smoothing
            # Then do PID tracking for NXT motors' pan-tilt control.
            #
            err_pan = exp_filter_pan(err_pan_i)
            err_tilt = exp_filter_tilt(err_tilt_i)
            control_pan = pid_pan(err_pan, pan_P, pan_I, pan_D)
            control_tilt = -1.0 * pid_tilt(err_tilt, tilt_P, tilt_I, tilt_D)
            
            #
            # Print out some data in a CSV compatible format for graphing
            #
            if not cvs_title_printed:
                print 'rel_time,err_tilt_i,err_tilt,control_tilt,err_pan_i,err_pan,control_pan'
                cvs_title_printed = True
               
            now = time.time() - ref_time 
            print '{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}'.format(now,err_tilt_i, err_tilt, control_tilt, err_pan_i, err_pan, control_pan)
            
            #
            # activate NXT motors
            #
            control_pan = power_limit(control_pan, 90.0)
            control_tilt = power_limit(control_tilt, 90.0)
            
            # uncomment one of the following lines
            # in order to isolate pan or tilt for PID tuning/testing
            #control_pan = 0.0
            #control_tilt = 0.0
            
            pan_motor.run(int(control_pan), True)
            tilt_motor.run(int(control_tilt), True)

        #
        # This section will turn motors off
        # when not in tracking mode. Note above code lines
        # force MODE_TRACK_HOLD if no objects exist or if more than one
        # object is detected. This state will shut motors off.
        #
        else:
            pan_motor.idle()
            tilt_motor.idle()

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
            batt_level = devNXT.get_battery_level() / 1000.0
            batt_level_text = '{:.2f} [v]'.format(batt_level)

        #
        # Add text and other markers to the image
        #
        if mode == MODE_TRACK_HOLD:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, mode_text, (1, 20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, tracker_type, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, fps_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        
        if batt_level > 6.50:
            cv2.putText(frame, batt_level_text, (1, 80), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, batt_level_text, (1, 80), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)

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
        elif key == ord('t'):
            mode_text = 'Track'
            if tracker_initialized:
                mode = MODE_TRACK
            else:
                mode = MODE_TRACK_HOLD
        elif key == ord('s'):
            mode_text = 'Show'
            mode = MODE_SHOW
        else:
            pass

    #
    # When done, stop motors and release the capture.
    #
    pan_motor.idle()
    tilt_motor.idle()
    cap.release()
    cv2.destroyAllWindows()

###########################################################
#
# exp_filter_pan()
#
#   Exponential filter for pan (X-axis) direction error.
#   Written as a function with an attribute.
#   TODO consider a class implementation
#
#   param:  current error, filter's alpha (default 0.1)
#   return: filtered error float value
#
def exp_filter_pan(err_i, alpha = 0.1):
    """Calculate Exponential filter for Pan error and returns filtered value."""
    filtered_err = (alpha * err_i) + ((1 - alpha) * exp_filter_pan.filtered_err_prev)
    exp_filter_pan.filtered_err_prev = filtered_err
    return filtered_err
#
exp_filter_pan.filtered_err_prev = 0.0

###########################################################
#
# exp_filter_tilt()
#
#   Exponential filter for tilt (Y-axis) direction error.
#   Written as a function with an attribute.
#   TODO consider a class implementation
#
#   param:  current error, filter's alpha (default 0.1)
#   return: filtered error float value
#
def exp_filter_tilt(err_i, alpha = 0.1):
    """Calculate Exponential filter for Tilt error and returns filtered value."""
    filtered_err = (alpha * err_i) + ((1 - alpha) * exp_filter_tilt.filtered_err_prev)
    exp_filter_tilt.filtered_err_prev = filtered_err
    return filtered_err
#
exp_filter_tilt.filtered_err_prev = 0.0

###########################################################
#
# pid_pan()
#
#   PID controller for pan (X-axis) direction error.
#   Written as a function with an attribute.
#   The function does its own time measurements between calls
#   to get delta time in mSec for controller calculations
#   TODO consider a class implementation
#
#   param: process error, Kp, Ki, Kd (PID constants) 
#   return: control output (as a float, and not limited!)
#
def pid_pan(error, Kp = 0.0, Ki = 0.0, Kd = 0.0):
    """Accepts position error and calculates PID control output for Pan direction"""
    e2 = cv2.getTickCount()
    delta_time = (e2 - pid_pan.e1) / cv2.getTickFrequency()
    pid_pan.e1 = e2
    pid_pan.integral = pid_pan.integral + (error * delta_time)
    derivative = (error - pid_pan.previous_error) / delta_time
    output = -1.0 * ((Kp * error) + (Ki * pid_pan.integral) + (Kd * derivative))
    pid_pan.previous_error = error
    return output
#
pid_pan.previous_error = 0.0
pid_pan.integral = 0.0
pid_pan.e1 = cv2.getTickCount()

###########################################################
#
# pid_tilt()
#
#   PID controller for tilt (Y-axis) direction error.
#   Written as a function with an attribute.
#   The function does its own time measurements between calls
#   to get delta time in mSec for controller calculations
#   TODO consider a class implementation
#
#   param: process error, Kp, Ki, Kd (PID constants) 
#   return: control output (as a float, and not limited!)
#
def pid_tilt(error, Kp = 0.0, Ki = 0.0, Kd = 0.0):
    """Accepts position error and calculates PID control output for Tilt direction"""
    e2 = cv2.getTickCount()
    delta_time = (e2 - pid_tilt.e1) / cv2.getTickFrequency()
    pid_tilt.e1 = e2
    pid_tilt.integral = pid_tilt.integral + (error * delta_time)
    derivative = (error - pid_tilt.previous_error) / delta_time
    output = +1.0 * ((Kp * error) + (Ki * pid_tilt.integral) + (Kd * derivative))
    pid_tilt.previous_error = error
    return output
#
pid_tilt.previous_error = 0.0
pid_tilt.integral = 0.0
pid_tilt.e1 = cv2.getTickCount()

###########################################################
#
# power_limit()
#
#   Will limit motor power to a clip level for NXT motors.
#
#   param:  input power, absolute clip level
#   return: clipped NXT motor power
#
def power_limit(power, clip_level = 100.0):
    """Compares input power level against a clip value, and returns a clipped value if level is exceeded."""
    if power > clip_level:
        output = clip_level
    elif power < -clip_level:
        output = -clip_level
    else:
        output = power
        
    return output

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

###############################################################################
# 
# blobtrack.py
#
#   Python OpenCV and Python Lego NXT implementing an object tracking
#   webcam mounted on a Lego pan-tilt device.
#   Using HSV color space for object color detection.
#
#   The program requires a connected webcam and an NXT pan-tilt mechanical setup
#   The program has three operating modes:
#   'm' key Mark: enter this mode while holding a uniformly colored object
#       in front of the webcam. High contrast colors seem to work best, with a
#       uniform background. Use the mouse to mark a region over the object,
#       which selects the color region to track as a blob.
#   's' key Show: this is a test mode that shows the tracked blob. Use this mode
#       to test blob tracking on the display.
#   't' key Track: this mode activates the motors and sends power controls to them.
#       The Track mode will be halted (red 'Track' symbol) if the blob is lost, or
#       more than one blob is detected. Tracking will resume automatically when
#       a single blob is re-detected.
#
#   January 13, 2018
#
###############################################################################

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
    """Control function that read webcam, tracks object, and controls NXT motors to follw using PID."""
    global x0, y0, x1, y1, drawing, frameHSV, mode, lower, upper
    global MODE_MARK

    #
    # initialization
    #
    MODE_TRACK = 0          # track an object
    MODE_SHOW = 1           # only show tracking markers on video
    MODE_MARK = 2           # mark region color to track
    MODE_TRACK_HOLD = 3     # temporarily suspend tracking until object is recaptured

    lower = np.array([0,0,0], dtype='uint8')
    upper = np.array([255,255,255], dtype='uint8')
    mode = MODE_SHOW
    mode_text = 'Show'
    fps_text = '?? Fps'
    batt_level = 0.0
    batt_level_text = '?? [v]'
    
    drawing = False         # true if mouse is pressed
    x0, y0 = -1, -1
    x1, y1 = -1, -1
    
    #
    # PID constants for pan and tilt PID controller.
    # These constants are tuned for the worm-gear mechanical setup
    #
    pan_P = 2.0
    pan_I = 0.1
    pan_D = 1.0
    
    tilt_P = 1.0
    tilt_I = 0.1
    tilt_D = 0.5
          
#
#   The PID parameters below are for non worm-gear mechanics.
#   Kept here for reference.
#  
#    pan_P = 0.25            # PID constants for pan and tilt PID controller
#    pan_I = 0.01
#    pan_D = 0.1
#    
#    tilt_P = 0.10
#    tilt_I = 0.01
#    tilt_D = 0.05

    print ' m - mark color region to track\n t - track\n s - display tracking marker only\n ESC - quit'

    #
    # connect to NXT device
    # define motor references
    #
    devNXT = nxt.locator.find_one_brick()
    tilt_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_A)
    pan_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_B)

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
    # kernel for dilation or erosion
    #
    dilate_kernel = np.ones((3, 3), np.uint8)
    erode_kernel = np.ones((5, 5), np.uint8)
    
    #
    # Set up a blob detector with some parameters
    #
    det_param = cv2.SimpleBlobDetector_Params()
    #det_param.thresholdStep = 1
    #det_param.minThreshold = 0
    #det_param.maxThreshold = 0
    #det_param.minRepeatability = 10
    #det_param.minDistBetweenBlobs = 10
    det_param.filterByColor = False
    det_param.filterByCircularity = False
    det_param.filterByInertia = False
    det_param.filterByConvexity = False
    det_param.filterByArea = True
    det_param.minArea = 500
    det_param.maxArea = 9000
    if cv2.__version__.startswith('3.'):
        detector = cv2.SimpleBlobDetector_create(det_param)
    else:
        detector = cv2.SimpleBlobDetector(det_param)

    #
    # open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)
        
        frameWidth = cap.get(cv.CV_CAP_PROP_FRAME_WIDTH)
        frameHeight = cap.get(cv.CV_CAP_PROP_FRAME_HEIGHT)
        
        print 'frame: width {}, height {}'.format(frameWidth, frameHeight)
        
        frameCenterX = int(frameWidth/2)
        frameCenterY = int(frameHeight/2)

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

        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #
        # operations on the captured image done here
        # if marking a section on the object color for tracking
        # then only display the selection
        #
        if mode == MODE_MARK:
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 1)

        #
        # if tracking or only displaying object tracking information
        # then draw the tracking markers on the frame before it is displayed
        #
        else:
            #
            # calculate tracking and show markers on video
            # create NumPy arrays from the 'upper' and lower' boundaries
            # source: https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
            # source: https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/
            #
            mask = cv2.inRange(frameHSV, lower, upper)

            #
            # find blob and calculate center of mass and deviation from 
            # center of frame. this will be the tracking error
            # source: https://www.learnopencv.com/blob-detection-using-opencv-python-c/
            #
            mask = cv2.erode(mask, erode_kernel, iterations = 1)
            mask = cv2.dilate(mask, dilate_kernel, iterations = 1)
            mask = cv2.bitwise_not(mask)
            keypoints = detector.detect(mask)

            #
            # draw detected blobs 
            #
            if len(keypoints) == 1:
                #
                # draw a vector from frame center to block center of mass
                # calculate 'X' and 'Y' errors from center where
                #
                blobX = int(keypoints[0].pt[0])
                blobY = int(keypoints[0].pt[1])
                err_pan_i = frameCenterX - blobX
                err_tilt_i = frameCenterY - blobY
                cv2.arrowedLine(frame, (frameCenterX, frameCenterY), (blobX, blobY), (0, 255, 0), 1)
                cv2.circle(frame, (blobX, blobY), int(keypoints[0].size), (0, 255, 0), 1)
                if mode == MODE_TRACK_HOLD:
                    mode = MODE_TRACK

            elif len(keypoints) == 0:
                #
                # red cross marker in center of frame if no
                # blobs were detected
                #
                cv2.drawMarker(frame, (frameCenterX, frameCenterY), (0, 0, 255), cv2.MARKER_TILTED_CROSS)
                if mode == MODE_TRACK:
                    mode = MODE_TRACK_HOLD

            else:
                #
                # draw all detected blobs if more than one
                # were detected; ambiguous detection
                #
                frame = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                if mode == MODE_TRACK:
                    mode = MODE_TRACK_HOLD
        
        #
        # Only when in tracking mode activate the motors,
        # and use PID calculations to control the pan-tilt device
        #
        if mode == MODE_TRACK:
            #
            # First apply an exponential filter to the blob position error.
            # info: https://en.wikipedia.org/wiki/Exponential_smoothing
            # Then do PID tracking for NXT motors' pan-tilt control.
            #
            err_pan = exp_filter_pan(err_pan_i)
            err_tilt = exp_filter_tilt(err_tilt_i)
            control_pan = pid_pan(err_pan, pan_P, pan_I, pan_D)
            control_tilt = -1.0 * pid_tilt(err_tilt, tilt_P, tilt_I, tilt_D)
            print 'tilt {:.2f}/{:.2f}/{:.2f} pan {:.2f}/{:.2f}/{:.2f}'.format(err_tilt_i, err_tilt, control_tilt, err_pan_i, err_pan, control_pan)
            
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
            
        cv2.putText(frame, fps_text, (1, 40), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        
        if batt_level > 6.50:
            cv2.putText(frame, batt_level_text, (1, 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
        else:
            cv2.putText(frame, batt_level_text, (1, 60), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
            
        #
        # Display the resulting frame
        #
        cv2.imshow('blob view', mask)
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
            mode = MODE_TRACK
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
#   return: nothing, calculates upper and lower HSV value vectors (globals)
#
def mark_rect(event, x, y, flags, param):
    """Mouse callback that calculates upper and lower HSV vectors for a marked frame region."""
    global x0, y0, x1, y1, drawing, frameHSV, mode, lower, upper
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
        # Extract ROI and calculate lower and upper RGB limit tuples
        #
        if x0 == x1 or y0 == y1 or x0 < 0 or x1 < 0 or y0 < 0 or y1 < 0:
            return
        if x0 > x1:
            x0, x1 = x1, x0
        if y0 > y1:
            y0, y1 = y1, y0

        roi = frameHSV[y0+1:y1, x0+1:x1]
        lower = np.array([np.amin(roi[:,:,0]), np.amin(roi[:,:,1]), np.amin(roi[:,:,2])], dtype='uint8')
        upper = np.array([np.amax(roi[:,:,0]), np.amax(roi[:,:,1]), np.amax(roi[:,:,2])], dtype='uint8')

#
# Startup
#
if __name__ == '__main__':
    main()

###############################################################################
# 
# pantilt.py
#
#   Manual test of pan-tilt Lego NXT setup for worm gear mechanics
#
#   January 13, 2018
#
###############################################################################

import numpy as np
import cv2
import opencvconst as cv

import nxt

def main():
    """Accepts [u]p [d]own [l]eft [r]ight key commands and runs NXT motors."""
    #
    # initializations
    #
    PAN_TILT_IDLE  = 0
    PAN_TILT_UP    = 1
    PAN_TILT_DOWN  = 2
    PAN_TILT_LEFT  = 3
    PAN_TILT_RIGHT = 4
    
    direction = PAN_TILT_IDLE
    motorsOn = True
    e1 = 0
    pan_power  = 0
    tilt_power = 0
    
    #
    # open the capture device and print some
    # useful properties
    #
    cap = cv2.VideoCapture(0)
    
    if cap.isOpened():
        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 240)

    #
    # connect to NXT device
    # define motor references
    #
    devNXT = nxt.locator.find_one_brick()
    tilt_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_A)
    pan_motor = nxt.motor.Motor(devNXT, nxt.motor.PORT_B)
    
    #
    # frame capture and processing loop
    #
    while(True):
        #
        # capture a frame
        # cover to appropriate color space to improve detection
        # in different lighting conditions
        #
        ret, frame = cap.read()

        #
        # display the resulting frame
        #
        cv2.imshow('Cam view', frame)
        
        #
        # activate pan-tile motors
        # from camera point of view (pan/tilt):
        #   - with new setup, tilt is tilt_motor only
        #     (0/+) up
        #     (0/-) down
        #   - with new setup, panning REQUIRES tilt_motor compensation
        #     because of gearing ratios, (pan_power:tilt_power) is about (5:1)
        #     (-/-) left
        #     (+/+) right
        #
        # original setup:       https://www.youtube.com/watch?v=NSRRrAH-9cA
        # new mechanical setup: https://www.eurobricks.com/forum/index.php?/forums/topic/83078-pantilt-mount-for-a-small-camera/
        #                       https://youtu.be/ei3JFqVvChU
        #
        if not motorsOn:
            #
            # flag that motors are on and start timing their run time
            # use run() with speed regulation, a default for the Motor() constructor, as True
            #
            motorsOn = True
            e1 = cv2.getTickCount()
            pan_motor.run(pan_power, True)
            tilt_motor.run(tilt_power, True)

        e2 = cv2.getTickCount()
        time = (e2 - e1) / cv2.getTickFrequency()

        #
        # brake() method seems to be more accurate
        # but idle() method may need to be used with
        # using this in object tracking
        #
        if time >= 1 and motorsOn == True:
            pan_motor.brake()
            tilt_motor.brake()
        
        #
        # key input mode/command
        #
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:
            break
        elif key == ord('u'):
            direction = PAN_TILT_UP
            pan_power = 0
            tilt_power = -50
            motorsOn = False
        elif key == ord('d'):
            direction = PAN_TILT_DOWN
            pan_power = 0
            tilt_power = +50
            motorsOn = False
        elif key == ord('l'):
            direction = PAN_TILT_LEFT
            pan_power = -60
            tilt_power = +12
            motorsOn = False
        elif key == ord('r'):
            direction = PAN_TILT_RIGHT
            pan_power = +60
            tilt_power = -12
            motorsOn = False
        else:
            pass

    #
    # when done, stop motors and release the capture
    #
    pan_motor.idle()
    tilt_motor.idle()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


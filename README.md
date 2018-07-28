# Object tracking with Lego NXT and OpenCV
Object tracking using a Web cam with OpenCV video processing.
Webcam is mounted on a Lego NXT driven pan-tilt.
Additional details here [https://sites.google.com/site/eyalabraham/computer-vision] and some video demos here [https://youtu.be/XIs69O7Js_U]

## Setup
### Lego NXT Development environment on Ubuntu
You can follow the steps outlined here [http://ubuntudaily.blogspot.com/2011/03/using-lego-mindstorms-nxt-with-ubuntu.html] and [http://bricxcc.sourceforge.net/nbc/doc/nxtlinux.txt], and then the installation of the Python libraries listed below.
However, these seems to be a bit outdated as most of these tools are included in the Ubuntu repositories (at least as of 16.04 LTS), and are available through standard Ubuntu package distribution.
Follow these steps to install the utilities and development tools you need:
1. Open Ubuntu Synaptic Package manager, or use 'apt-get'
2. Install: nbc, t2n
3. Optional install: libnxt, python-nxt, python-nxt-filer
4. Configuring USB access privileges was not necessary, things worked as is on Ubuntu 16.04 LTS

### Eclipse or gEdit?
I ended up using a Python based project. The simplest solution was to use gEdit

### OpenCV
Follow [https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/] or [https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html] for OpenCV setup on Ubuntu 16.04
Download latest OpenCV distribution from [https://opencv.org/] release 3.4.0
OpenCV Python wiki at [https://docs.opencv.org/3.4.0/d6/d00/tutorial_py_root.html]
Good resource for learning the basics of object detection and computer vision [http://www.learnopencv.com/]
To get support for OpenCV tracker functionality follow [https://pypi.python.org/pypi/opencv-contrib-python]

### Mechanical
A few options exist for the Pan-Tilt mechanical setup. There is a nice Differential Pan & Tilt: [https://www.youtube.com/watch?v=NSRRrAH-9cA] that I built first, but, being made out of Lego, it proved not to be robust enough. The backlash from the motors and the gears, especially the horizontal one, was excessive. The vertical axis was too 'wobbly' to hold a webcam.
I opted for this arrangement [https://www.youtube.com/watch?v=ei3JFqVvChU]. The tilt movement when panning was not an issue because the tilt PID took care of any error. The mechanical arrangement was much 'tighter' and backlash was minimal due to better gearing.
There are many other arrangements to be found, but these were the most elegant ones I found with Lego.

## Resources
### Python NXT interfacing
- nxt-python for Python 2.x [https://github.com/Eelviny/nxt-python/releases]
- Examples for nxt-python: [https://github.com/MiltonStatic5060/nxt-comp-app]
- NXT SDK downloads: [https://www.lego.com/en-us/mindstorms/downloads/?domainredir=mindstorms.lego.com]

### General NXT programing
- **Main resource** NXC and NBC programing: [http://bricxcc.sourceforge.net/nbc/]
- **API documentation** [http://bricxcc.sourceforge.net/nbc/nxcdoc/nxcapi/index.html]
- NXT programming options: [http://www.teamhassenplug.org/NXT/NXTSoftware.html]
- NXC programing: [http://ubuntudaily.blogspot.com/2011/03/using-lego-mindstorms-nxt-with-ubuntu.html]
- Direct access library code examples: [www.mindsqualls.net/Default.aspx]

### NXT motor control
- NXT Motor Control:
-- [http://www.mindstorms.rwth-aachen.de/trac]
-- [http://www.mindstorms.rwth-aachen.de/trac/wiki/MotorControl]
-- [http://www.mindstorms.rwth-aachen.de/trac/wiki/Download]
-- [http://www.mindstorms.rwth-aachen.de/documents/downloads/doc/version-4.03/motor_control.html]
-- [http://www.mindstorms.rwth-aachen.de/trac/wiki/Documentation]
-- [http://www.mindstorms.rwth-aachen.de/trac/wiki/FAQ]
- Projects:
-- [http://jander.me.uk/LEGO/]
-- [http://r.jander.me.uk/]
-- [http://www.cs.tau.ac.il/~stoledo/lego/]

## Performance observations
Basic profiling using carefully placed `cv2.getTickCount()` yielded the following observations:
1. Measuring the entire control loop, including video capture, shows a gradual increase in loop time up to about 0.25sec(!)
2. Separate measurements on the `cap.read()` command vs. the rest of the control loop shows that all the control loop processing is stable around 0.015sec, while `cap.read()` command slowly slows down to 0.25sec
3. using the FPS object from imutils an average of 4 frames per second was measured given the method described in resource #1 below.

### Approach #1: implemented the threaded frame grab from the webcam
A web search yielded the following resource dealing with cv2.VideoCapture(0) and cap.read() image streaming throughput:
1. Faster video file FPS with cv2.VideoCapture and OpenCV [https://www.pyimagesearch.com/2017/02/06/faster-video-file-fps-with-cv2-videocapture-and-opencv/]
2. Increasing webcam FPS with Python and OpenCV [https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/]

Implemented the frame grab using the example in resource #2 above.
The frame rate increased and was stable at around 16 FPS (0.06sec per frame).
However, other issues appeared:
1. Following the implementation in imutils it looks like an artificial increase in FPS from the webcam.
   The webcam does not actually increase its FPS, the thread simply feeds stale frames whenever asked for one.
2. Some of the comments in the blog seem to indicate the same, so I abandoned this solution.

A webcam read thread is not the solution.

### Approach #2: webcam behavior
Some resources indicate that webcam FPS is influenced by several variable including: USB throughput limit, subject lighting, frame format. I cannot change USB throughput because the webcam is a given device on USB 2.0. However, the other two variable may be under my control.

To validate FPS changes in different lighting conditions I used a basic capture loop and measured FPS in different lighting situations. First I tried to read the webcam FPS using `cv2.get(cv.CV_CAP_PROP_FPS)`. This returned NaN; interesting! Setting with `cv2.get(cv.CV_CAP_PROP_FPS, <some_rate>)` did nothing.
The results were as follows for FPS samples every 10sec:

 FPS    |   Lighting
--------|------------
 8.14   |   Start of program
 7.00   |
 5.21   |   Gradual decline in FPS
 4.32   |
 3.78   |
 3.42   |
 3.18   |
 3.18   |   Minimum FPS in low light
 3.23   |
 3.52   |   Turned all lights on
 3.94   |   Subject is well lit
 4.56   |   FPS increases!
 4.99   |
 4.99   |
 4.99   |
 4.97   |
 4.99   |
 4.99   |
 4.99   |
 4.83   |   Turned lights off again
 4.17   |   FPS decreases
 3.68   |
 3.33   |
 3.06   |
 
### Approach #3: swap webcam for a newer model
Newer webcam model provided a frame rate of 27FPS in high lighting conditions and minimum of 15FPC in low, near dark, conditions.
The solution was to swap the old webcam for the new model. The new model was a bit heavier so I increased the gearing ration on the tilt mechanism. The gearing change and the faster, stable, frame rate did the trick!

## Application notes
### Track object as a color blob blobtrack.py
Python OpenCV and Python Lego NXT implementing an object tracking webcam mounted on a Lego pan-tilt device. It is based on [https://www.learnopencv.com/blob-detection-using-opencv-python-c/]
The blob tracker uses HSV color space for object color detection.
The program requires a connected webcam and an NXT pan-tilt mechanical setup and has three operating modes:
'm' key **Mark**: enter this mode while holding a uniformly colored object in front of the webcam. High contrast colors seem to work best, with a uniform background. Use the mouse to mark a region over the object, which selects the color region to track as a blob.
's' key **Show**: this is a test mode that shows the tracked blob. Use this mode to test blob tracking on the display.
't' key **Track**: this mode activates the motors and sends power controls to them. The Track mode will be halted (red 'Track' symbol) if the blob is lost, or more than one blob is detected. Tracking will resume automatically when a single blob is re-detected.

### Track object with tracking algorithms objtracker.py
Python OpenCV and Lego NXT program for tracking object with a webcam and a pan-tilt system. It is based on code from [https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/]
The program requires a connected webcam and an NXT pan-tilt mechanical setup and has three operating modes:
'm' key **Mark**: enter this mode while holding an object in front of the webcam, and use the mouse to select it with a rectangle.
's' key **Show**: this is a test mode that shows the tracked object. Use this mode to test tracking on the display.
't' key **Track**: this mode activates the motors and sends power controls to them. The Track mode will be halted and a red bounding box is displayed if the object is lost. Tracking will resume automatically when the tracker algorithm re-detects the object. With the default selected tracker algorithm set to KCF simply bringing the object back into the the view of the red bounding box will usually resume detection.

### Room enter and exit monitor based on sensing movement with video frame diff peoplecount_diff.py
This implementation uses a webcam pointed across the room entry way, and counts people entering and exiting the room. It uses a simple frame subtraction and threshold method to find differences between consecutive frames. The differences are interpreted as motion. The delta image is thresholded and then searched for contours.
Because there may be many contours, the program finds the bounding box for *all* contours, and then marks that 'super' bounding box on the video.
The 'super' contour is tracked vs. two vertical lines on the video frame, and a pattern of crossing both lined in one direction is counted as entry or exit from the room.
Because the camera is not mounted from above, counting might not be accurate.

### PID tuning
The pan-tilt system has two PID controllers, one for Pan and another for Tilt. Both controllers calculate the control value that is then fed to the NXT motors. Tuning the PID coefficients Kp, Ki, and Kd can be done by following any number of methods. After the initial setting good deal of trial and error can hone them to achieve the desired response.

When tuning, the physical properties of the mechanical setup should be taken into account. The following is a summary of the main ideas:
- The NXT motors seem to run at low efficiency when run at low power/speed. The experienced uses state that a minimum power level of +/-50 should be used. In blobtrack.py I let the controller set the power levels even if they were much lower, and seemed to achieve reasonable performance. However, I plan to increase the gearing ratio with a worm gear, which may help drive the power set-point higher, hopefully yielding even better tracking performance. I rebuilt the base driver with worm gears that allowed running the motors at higher speeds.
After modifying the gear train to include a worm-gear I was able to achieve smoother and more responsive tracking. This change allowed the control loop to drive the motors with higher power settings, which in turn provided better motion startup response and higher torque.
- The pan direction is a PD controller. The gearing ratio and the low speeds were sufficient to avoid excessive steady state errors, so no need for an Integrator component. However, some dampening was needed to prevent overshoots.
- The tilt direction is a PID controller. The gearing ratio is lower, and the inertia produced by the webcam seems to be more significant in this direction. The Differential component offers dampening for overshoots (although it can be increased). The Integral component serves two functions:
First, it helps correct small residual tilt position errors after a move, and second, it helps correct tilt errors after a panning movement. Due to gearing, panning produces a small tilt movement that the tilt Integrator of the PID corrects.
- When using higher frame resolutions such as 640x480 instead of 320x240, the errors double in amplitude. Care should be taken to reduce the PID constants by at least the same ratio so that the controller will not saturate its output.

## Files
*README.md* This file

*blobtrack.py* Color blob object detection and tracking

*objtracker.py* Object tracking with KCF tracker with pan-tilt camera mount

*peoplecount_diff.py* Room enter and exit monitor based on sensing movement with video frame diff

*expfilter.py* A class implementing an exponential filter object
*opencvconst.py* Some OpenCV camera property constants
*cspace.py* Test program for checking out color space conversions
*blobtrackHSV.py* Testing HSV color conversation blob detection
*blobtrackRGB.py* Testing RGB color conversation blob detection
*pantilt.py* Test driver for NXT motor motion in pan-tilt setup
*webcamperf.py* Test program for webcam Fps performance
*trackers.py* Test program for comparing various tracker algorithm behavior

*opticalflow1.py* study optical flow using Lucas-Kanade Optical Flow method for sparse points of interest
*opticalflow2.py* study optical flow using Dense Optical Flow method
*opticalflow3.py* study optical flow study Shi-Tomasi Corner Detector
*opticalflow4.py* Lucas-Kanade Optical Flow method for sparse points of interest track predefines a set of points on the source image
*opticalflow5.py* study optical flow using sequential frame subtraction method
*peoplecount_lk.py* program to count people entering and exiting a room using Lucas-Kanade Optical Flow method



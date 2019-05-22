# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import argparse
import imutils
import serial
 

#serial transports
ser = serial.Serial('/dev/ttyUSB0',115200)

#Location
c1=0
c2=0
c3=0
c4=0

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (320, 320)
camera.framerate = 40
#camera.hflip = True

rawCapture = PiRGBArray(camera, size=(320, 320))
 
# allow the camera to warmup
time.sleep(2.0)

# define the lower and upper boundaries of the "green"
# ball in the HSV color space
greenLower = (29, 86, 10)
greenUpper = (64, 255, 255)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    blur = cv2.GaussianBlur(image, (11, 11), 0)
##    blur = cv2.blur(image, (3,3))

    #hsv to complicate things, or stick with BGR
    #hsv = cv2.cvtColor(blur,cv2.COLOR_BGR2HSV)
    #thresh = cv2.inRange(hsv,np.array((0, 200, 200)), np.array((20, 255, 255)))

    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=5)
    mask = cv2.dilate(mask, None, iterations=2)
    
    
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
            
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        
        a,b,w,h = cv2.boundingRect(c)
        aspect_ratio = float(w)/h
        print(radius)
        print("\t")
        print(aspect_ratio)
        print("\n")
        
        # only proceed if the radius meets a minimum size
        if (radius > 10): ##and ((aspect_ratio >= 0.5) and (aspect_ratio <= 2))):
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(image, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            #cv2.circle(blur, center, 5, (0, 0, 255), -1)
            M = cv2.moments(c)
            c1 = int(M["m10"] / M["m00"])
            c2 = 320-int(M["m01"] / M["m00"])
            
            if c1 < 5:
                c1 = 5
            if c1 > 315:
                c1 = 315
            
            if c2 < 5:
                c1 = 5
            if c2 > 315:
                c2 = 315
                
            if c1 >= 0 and c1 < 80:
                c3 = 1
            elif c1 >= 80 and c1 < 125:
                c3 = 2
            elif c1 >= 125 and c1 < 158:
                c3 = 3
            elif c1 >= 158 and c1 <= 162:
                c3 = 4
            elif c1 > 162 and c1 <= 195:
                c3 = 5
            elif c1 > 195 and c1 <= 240:
                c3 = 6
            elif c1 > 240 and c1 <= 320:
                c3 = 7
                
            if c2 >= 0 and c2 < 80:
                c4 = 1
            elif c2 >= 80 and c2 < 125:
                c4 = 2
            elif c2 >= 125 and c2 < 158:
                c4 = 3
            elif c2 >= 158 and c2 <= 162:
                c4 = 4
            elif c2 > 162 and c2 <= 195:
                c4 = 5
            elif c2 > 195 and c2 <= 240:
                c4 = 6
            elif c2 > 240 and c2 <= 320:
                c4 = 7
                
        else:
            c1 = 0
            c2 = 0
            c3 = 0
            c4 = 0
    else:
        c1 = 0
        c2 = 0
        c3 = 0
        c4 = 0
            
    #Send data
    send = str(c3) + str(c4)                     
    ser.write(send.encode())
    time.sleep(0.1)
    
##    send = str(c1) + str(c2)                     
##    ser.write(send.encode())
##    time.sleep(0.1)
    cv2.circle(image,(c1,320 - c2), 5, (0,0,255), -1)
    cv2.putText(image, "c1: {}, c2: {}".format(c1, c2),
        (10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
        0.35, (0, 0, 255), 1)
        
    # show the frame
    cv2.imshow("image", image)
    cv2.imshow('thresh',mask)
    
    key = cv2.waitKey(1) & 0xFF
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        ser.write(b'99')
        time.sleep(1)
        break


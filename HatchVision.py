import numpy
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
#from networktables import NetworkTable
import pistream
import sys
import math


# if running from command line, passing any value as a parameter will
# stop the image windows from being displayed.
if len(sys.argv) > 1:  #python filename is always in arg list, so look for 2 or more
    showImages = False
else:
    showImages = True

LOWER_HSV = numpy.array([43,113,130])
UPPER_HSV = numpy.array([99,255,255])

#These constants control goal detection values


# Setup PiCamera for capture
# IMG_WIDTH = 240
# IMG_HEIGHT = 180
IMG_WIDTH = 640
IMG_HEIGHT = 368
IMG_CENTER_ERROR = 0

GEAR_STATE = 0
BOILER_STATE = 1
video = pistream.PiVideoStream((IMG_WIDTH,IMG_HEIGHT)).start()
time.sleep(2)  #Give camera a chance to stablize





def dst2errorX(TargetCenter, Distance) :
    # theta = math.atan(TargetCenter/581.4)
    theta = math.atan((TargetCenter-IMG_CENTER_ERROR)/622.7722)
    return Distance*math.sin(theta)




# Process images continuously, outputting a comamnd to the robot each cycle
def process():
    # Init contour measurements for picture saving
    h = 0
    w = 0
    center = 0
    imageCounter = 0  #Counter for filename of saved images

    kernel = numpy.ones((3,3),
                        numpy.uint8)  #used to do dialate

    dist = [0.0,0.0]
    angle = 0.0 # target angle in degrees
    fps = 0
    a = 0

    while True:
        startTime = time.time()
        #print("Working "+str(startTime))

        # Grab frame from PiCamera
        img_bgr = video.read()

        # Resize to a smaller image for better performance
        img_bgr = cv2.resize(img_bgr, (IMG_WIDTH,IMG_HEIGHT),interpolation = cv2.INTER_LINEAR)


        img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

        # Create a binary threshold based on the HSV range (look for green reflective tape)
        img_thresh = cv2.inRange(img_hsv, LOWER_HSV, UPPER_HSV)
        #if showImages:
        #    cv2.imshow("img_thresh",img_thresh)

        # Dilate them image to fill holes
        img_dialate = cv2.dilate(img_thresh, kernel, iterations=1)

        if showImages:
            cv2.imshow("img_dialate",img_dialate)

        # Find all of the contours in binary image
        _, contours, _ = cv2.findContours(img_dialate, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Now we'll evaluate the contours.
        # We'll assume that no goal is visible at first.
        # The value of 'turn' will indicate what action the robot should take.
        # 0=no goal, do nothing, 999=aligned with goal, shoot, +/-x.xxx=turn rate to move to target
        turn = 0
        okCount = 0
        j = 0
        OkCountours = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        img_rect = img = numpy.zeros((IMG_HEIGHT, IMG_WIDTH,3), numpy.uint8)
        print("# of contours: " +str(len(contours)))

        
        #Check all contours, mark ones that may look like a target
        if len(contours) > 0:
            for c in contours:
                # Is the contour big enough to be a target?
                #area = cv2.contourArea(c) # area of rectangle that is filled
                x,y,w,h = cv2.boundingRect(c)
                
                ratio = float(h) / float(w)
                #5.826/3.313 = 1.758527
                ratioError = ratio - (5.826/3.313)
                #print (h, w, w*h, ratio)
                if abs(ratioError) < 1 and  w*h > 100:
                    ((x_t, y_t), (w_t, h_t), angle) = cv2.minAreaRect(c)
                    #angle is negative, -90 is the same as -0
                    #right leaning is -0 to -20
                    #left learning is -90 to -70
                    #angle of targets is 14.5 and 75.5
                    print(angle)
                    #center = x + (w/2)
                    if okCount > 11:
                        print("Too much contours")
                        break
                    if showImages: 
                        cv2.rectangle(img_rect, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    OkCountours[okCount] = j
                    okCount = okCount + 1
                j = j +1
        else:
            print ("No contours found")
        # print "RatioOkCount:",okCount
        i = 0
        markercount = 0
        Yr = [0,0]
        Xr = [0,0]
        pixAngle = 0
        while  okCount > i+1:
            #print("okcountour")
            x,y,w,h = cv2.boundingRect(contours[OkCountours[i]])
            """j = 0
            while i+j < okCount :
                x_p,y_p,w_p,h_p = cv2.boundingRect(contours[OkCountours[i+j]])
            j = j +1"""
            i = i+1


        if showImages:
            cv2.imshow("img_bgr",img_bgr)
            cv2.imshow("img_rect", img_rect)


        # Wait 1ms for keypress. q to quit.
        if cv2.waitKey(1) &0xFF == ord('q'):
            break

        fps = 1.0/(time.time()-startTime)


# Process until user exit (q or ctrl-c)
try:
    print("Starting hatch vision")
    process()
# Always clean up before exit
finally:
    video.stop()
    cv2.destroyAllWindows()

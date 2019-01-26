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

#Actual targets
REAL_HEIGHT = 5.826
REAL_WIDTH = 3.313
REAL_DISTANCE_BETWEEN = 8
REAL_RATIO = (REAL_HEIGHT/REAL_WIDTH)
REAL_PAIR_RATIO = REAL_WIDTH/(REAL_WIDTH+REAL_DISTANCE_BETWEEN)

#Should be 75.5
MIN_LEFT_ANGLE = 50
MAX_LEFT_ANGLE = 85

#Should be 14.5
MIN_RIGHT_ANGLE = 5
MAX_RIGHT_ANGLE = 30

GEAR_STATE = 0
BOILER_STATE = 1
video = pistream.PiVideoStream((IMG_WIDTH,IMG_HEIGHT)).start()
time.sleep(2)  #Give camera a chance to stablize




"""
    Not Changed
"""
def dst2errorX(TargetCenter, Distance) :
    # theta = math.atan(TargetCenter/581.4)
    theta = math.atan((TargetCenter-IMG_CENTER_ERROR)/622.7722)
    return Distance*math.sin(theta)

def pause() :
    print("pause")
    x = input()


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
        #turn = 0
        #okCount = 0
        #j = 0
        #OkCountours = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        rightContours = []
        #rightCount = 0
        leftContours = []
        #leftContours =  [0] * 11
        #leftCount = 0
        img_rect = img = numpy.zeros((IMG_HEIGHT, IMG_WIDTH,3), numpy.uint8)
        #print("# of contours: " +str(len(contours)))

        
        #Check all contours, mark ones that may look like a target
        if len(contours) > 0:
            #index of contour
            #z = 0
            for c in contours:
                # Is the contour big enough to be a target?
                #area = cv2.contourArea(c) # area of rectangle that is filled
                x,y,w,h = cv2.boundingRect(c)
                
                ratio = float(h) / float(w)
                #5.826/3.313 = 1.758527
                ratioError = ratio - REAL_RATIO
                #print (h, w, w*h, ratio)
                if abs(ratioError) < 1 and  w*h > 100:
                    tiltedRect = cv2.minAreaRect(c)
                    angle = abs(tiltedRect[2])
                    #((x_t, y_t), (w_t, h_t), angle) = tiltedRect
                    #angle is negative, -90 is the same as -0
                    #right leaning is -0 to -20
                    #left learning is -90 to -70
                    #angle of targets is 14.5 and 75.5
                    #print("Angle of possibles: "+ str(angle))
                    if angle >= MIN_RIGHT_ANGLE and angle <= MAX_RIGHT_ANGLE:
                        #print("right X + Y: "+str(tiltedRect[0])+" angle: "+str(angle))
                        print("right ", tiltedRect)
                        #if rightCount >= 11:
                        #    print("Too much contours")
                        #    break
                        if showImages: 
                            cv2.rectangle(img_rect, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        rightContours.append(tiltedRect)
                        #rightContours.append(z)
                        #rightCount += 1
                        #OkCountours[okCount] = j
                        #okCount = okCount + 1
                    elif angle >= MIN_LEFT_ANGLE and angle <= MAX_LEFT_ANGLE:
                        #print("lef X + Y: "+str(tiltedRect[0])+" angle: "+str(angle))
                        print("left ", tiltedRect)
                        #if leftCount >= 11:
                        #    print("Too much contours")
                        #    break
                        if showImages:
                            cv2.rectangle(img_rect, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        leftContours.append(tiltedRect)
                        #leftContours[leftCount] = z
                        #leftCount += 1
                        #okCount += 1
                    else:
                        print("not accepted", tiltedRect)
                #z+= 1
            #print("left count is: "+str(leftCount)+ " right count is: "+str(rightCount))
            """q = 0           
            while q<len(rightContours):
                print("right# "+str(rightContours[q]))
                q+= 1

            q = 0           
            while q<len(leftContours):
                print("left# "+str(leftContours[q]))
                q+= 1
            """

            if showImages:
                cv2.imshow("img_rect_mid", img_rect)
            pause()
            
            #print("# of rightContours ", len(rightContours), " # of leftContours ", len(leftContours))
            """iflen(rightContours) != 0 and len(leftContours) != 0:
                #print(str(rightCount)+" "+str(leftCount))
                print("working")"""
            i = 0
            while i < len(rightContours):
                (x_r,y_r),(w_r,h_r), angle_r = rightContours[i]
                #print("right contour: " + str(rightContours[i]))
                j = 0
                while j < len(leftContours):
                    (x_l, y_l), (w_l, h_l), angle_l = leftContours[j]
                    #print("left contour: "+ str(leftContours[j]))
                    # real length of target/distance between
                    # 3.313/8
                    # unsure which height to check
                    distanceBetween = abs(x_l-x_r)
                    if distanceBetween == 0 :
                        print("Distance between is 0")
                        print("left x is "+str(x_l))
                        print("right x is "+str(x_r))
                        #distanceBetween = 100
                        break
                    pairError = (w_r/distanceBetween) - REAL_PAIR_RATIO
                    if pairError < 1: 
                        print("okay pair"+str(i)+" "+str(j))
                        print("midpoint", (x_r-x_l)/2)
                    j+= 1
                i+= 1
                    
        else:
            print ("No contours found")
        #print ("OkCount: ", okCount)


        if showImages:
            cv2.imshow("img_bgr",img_bgr)
            # Commented for testing
            #cv2.imshow("img_rect", img_rect)
            #

        

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

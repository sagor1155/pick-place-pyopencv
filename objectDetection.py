
import numpy as np
import cv2
import time
import threading
from serialDev import commClass
#from imgprocess import imageprocess
#from matplotlib import pyplot as plt

#RED Threshold (nice)
lower_red = np.array([0, 120, 50])
upper_red = np.array([50, 255, 255])

#GREEN Threshold (nice)
lower_green = np.array([30,150,50])
upper_green = np.array([100,255,180])

# define range of red/green/blue color in HSV
lower_rgb = np.array([30,150,50])
upper_rgb = np.array([255,255,180])

color = "r"    # "r" for RED, "g" for GREEN, "b" for BLUE and "rgb" for RED/GREEN/BLUE
posBand = 15

 
def detectColorObject(rgbImage, lowerThreshold, upperThreshold):
    
    frame = cv2.medianBlur(rgbImage,5)
    # change color space of the frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = cv2.medianBlur(hsv,5)
        
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lowerThreshold, upperThreshold)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    #mask = cv2.GaussianBlur(res,(5,5),0)
    #ret,thresh1 = cv2.threshold(res, 100, 255, cv2.THRESH_BINARY)
    #img  = cv2.cvtColor(mask, cv2.COLOR_HSV2RGB)
    #img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    #th2  = cv2.adaptiveThreshold(img2,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)

    kernel   = np.ones((5,5), np.uint8)
    erosion  = cv2.erode(res, kernel, iterations = 1)
    dilation = cv2.dilate(erosion, kernel, iterations = 1)

    opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    imageRGB  = cv2.cvtColor(closing, cv2.COLOR_HSV2RGB)
    imageGRAY = cv2.cvtColor(imageRGB, cv2.COLOR_RGB2GRAY)
    
    #cv2.imshow('res', res)
    #cv2.imshow('erosion', erosion)
    #cv2.imshow('dilation', dilation)
    #cv2.imshow('opening', opening)
    cv2.imshow('closing', closing)
    cv2.waitKey(1)
        
    return imageGRAY


def getObjectPosition(mainImage, processedImage):
    # findContours returns a list of the outlines of the white shapes in the mask (and a heirarchy that we shall ignore)
    image, contours, hierarchy = cv2.findContours(processedImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
         
    # If we have at least one contour, look through each one and pick the biggest
    if len(contours)>0:
        largest = 0
        area = 0
        for i in range(len(contours)):
            # get the area of the ith contour
            temp_area = cv2.contourArea(contours[i])
            # if it is the biggest we have seen, keep it
            if temp_area > area:
                area = temp_area
                largest = i
        # Compute the coordinates of the center of the largest contour
        coordinates = cv2.moments(contours[largest])
        #print str(coordinates)
        if coordinates['m00'] != 0:
            target_x = int(coordinates['m10']/coordinates['m00'])
            print "Center X-coordinate: " + str(target_x)
            target_y = int(coordinates['m01']/coordinates['m00'])
            print "Center Y-coordinate: " + str(target_y)
            # Pick a suitable diameter for our target (grows with the contour)
            diam = int(np.sqrt(area)/4)
            print "diameter is: " + str(diam)
            # draw on a target
            cv2.circle(mainImage,(target_x,target_y),diam,(0,255,0),1)
            cv2.line(mainImage,(target_x-2*diam,target_y),(target_x+2*diam,target_y),(0,255,0),1)
            cv2.line(mainImage,(target_x,target_y-2*diam),(target_x,target_y+2*diam),(0,255,0),1)
            cv2.waitKey(1)
            return (target_x, target_y, diam)
    return (0, 0, 0)    


def display(txt):
    print txt

def readDataThread():

    while(True):
        try:
            data = "thread testing"
            print data
        except:
            print "Error"

if __name__ == "__main__":

    #imgObject = imageprocess()
 
    #t1 = threading.Thread(target=readDataThread)
    #t1.start()
    #t1.join()
##    try:
##        ardcom = commClass()
##        ardcom.serialConnect("COM7", 9600)
##    except:
##        print "can't connect"
##        time.sleep(10)
    vid = cv2.VideoCapture(0)
    objDetected = False
    while(True):
     
        if cv2.waitKey(1) & 0xFF == ord('a'):
            break
        else: 
            # Capture frame-by-frame
            #ardcom.sendString("capturing")
            ret, frame = vid.read()
            cv2.imshow('frame', frame)
            if objDetected == False:
                
                if color == "r":
                    try:
                        processedImage = detectColorObject(frame, lower_red, upper_red)
                        x, y, d = getObjectPosition(frame, processedImage)
                    except Exception, e:
                        print "Error Ocurred"
                        print str(e)
                        vid.release()
                        cv2.destroyAllWindows()
                elif color == "g":
                    try:
                        processedImage = detectColorObject(frame, lower_green, upper_green)
                        x, y, d = getObjectPosition(frame, processedImage)
                    except:
                        print "Error Ocurred"
                        vid.release()
                        cv2.destroyAllWindows()
                elif color == "rgb":
                    try:
                        processedImage = detectColorObject(frame, lower_rgb, upper_rgb)
                        x, y, d = getObjectPosition(frame, processedImage)
                    except:
                        print "Error Ocurred"
                        vid.release()
                        cv2.destroyAllWindows()

                print "printing diameter:  " + str(d)
                if d>5 and d<10:
                    #objDetected = True
                    print "Object Detected at position: (" + str(x) + "," + str(y) + ")"
##                    print "I am searching for Robotic ARM ..............."
##                    cv2.destroyAllWindows()
##                    time.sleep(1)
##                    for i in range(0, 1, 10):
##                        ret, frame = vid.read()
##                        cv2.imshow('frame', frame)
##                        cv2.waitKey(1)
##                        
##                   ##alt+3 for comment section & alt+4 for uncomment section   
##                    while(objDetected == True):
##                        ret, frame = vid.read()
##                        cv2.circle(frame,(x,y), (d*2) ,(0,255,0),1)
##                        cv2.line(frame,(x-2*d,y),(x+2*d,y),(0,255,0),1)
##                        cv2.line(frame,(x,y-2*d),(x,y+2*d),(0,255,0),1)
##                        cv2.imshow('frame', frame)
##                        cv2.waitKey(1)
##                        processedImage = detectColorObject(frame, lower_red, upper_red)
##                        cx, cy, dm = getObjectPosition(frame, processedImage)
##                        #time.sleep(5)
##                        print "Robot position: (" + str(cx) + "," + str(cy) + ")"
##                        if cx>(x-posBand) and cx<(x+posBand):
##                            print "X axis position matched"
##                            #time.sleep(1)
##                        if cy>(y-posBand) and cy<(y+posBand):
##                            print "Y axis position matched"
##                            #time.sleep(1)
##                        if cx>(x-posBand) and cx<(x+posBand) and cy>(y-posBand) and cy<(y+posBand):
##                            print "Both Axis Position Matched"
##                            objDetected = False
##                            cv2.destroyAllWindows()
##                            time.sleep(1)                        
##            

    # When everything done, release the capture
    vid.release()
    cv2.destroyAllWindows()


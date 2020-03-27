from picamera.array import PiRGBArray     #As there is a resolution problem in raspberry pi, will not be able to capture frames by VideoCapture
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import cv2
import cv2.cv as cv
import numpy as np


GPIO.setmode(GPIO.BOARD)

TRIGGERL = 29      
ECHOL = 31

TRIGGERF = 36      
ECHOF = 37

TRIGGERR = 33      
ECHOR = 35


MOTORL1=18  #Left Motor
MOTORL2=22

MOTORR1=19  #Right Motor
MOTORR2=21




# Set pins as output and input
GPIO.setup(TRIGGERL,GPIO.OUT)  # Trigger
GPIO.setup(ECHOL,GPIO.IN)      # Echo
GPIO.setup(TRIGGERF,GPIO.OUT)  # Trigger
GPIO.setup(ECHOF,GPIO.IN)      # Echo
GPIO.setup(TRIGGERR,GPIO.OUT)  # Trigger
GPIO.setup(ECHOR,GPIO.IN)      # Echo

# Set trigger to False (Low)
GPIO.output(TRIGGERL, False)
GPIO.output(TRIGGERF, False)
GPIO.output(TRIGGERR, False)

# Allow module to settle
def sonar(TRIGGER,ECHO):
      start=0
      stop=0
      # Set pins as output and input
      GPIO.setup(TRIGGER,GPIO.OUT)  # Trigger
      GPIO.setup(ECHO,GPIO.IN)      # Echo
     
      # Set trigger to False (Low)
      GPIO.output(TRIGGER, False)
     
      # Allow module to settle
      time.sleep(0.01)
           
      #while distance > 5:
      #Send 10us pulse to trigger
      GPIO.output(TRIGGER, True)
      time.sleep(0.00001)
      GPIO.output(TRIGGER, False)
      begin = time.time()
      while GPIO.input(ECHO)==0 and time.time()<begin+0.05:
            start = time.time()
     
      while GPIO.input(ECHO)==1 and time.time()<begin+0.1:
            stop = time.time()
     
      # Calculate pulse length
      elapsed = stop-start
      # Distance pulse travelled in that time is time
      # multiplied by the speed of sound (cm/s)
      distance = elapsed * 34000
     
      # That was the distance there and back so halve the value
      distance = distance / 2
     
      print "Distance : %.1f" % distance
      # Reset GPIO settings
      return distance

GPIO.setup(MOTORL1, GPIO.OUT)
GPIO.setup(MOTORL2, GPIO.OUT)

GPIO.setup(MOTORR1, GPIO.OUT)
GPIO.setup(MOTORR2, GPIO.OUT)

def forward():
      GPIO.output(MOTORL1, GPIO.HIGH)
      GPIO.output(MOTORL2, GPIO.LOW)
      GPIO.output(MOTORR1, GPIO.HIGH)
      GPIO.output(MOTORR2, GPIO.LOW)
     
def reverse():
      GPIO.output(MOTORL1, GPIO.LOW)
      GPIO.output(MOTORL2, GPIO.HIGH)
      GPIO.output(MOTORR1, GPIO.LOW)
      GPIO.output(MOTORR2, GPIO.HIGH)
     
def rightturn():
      GPIO.output(MOTORL1,GPIO.LOW)
      GPIO.output(MOTORL2,GPIO.HIGH)
      GPIO.output(MOTORR1,GPIO.HIGH)
      GPIO.output(MOTORR2,GPIO.LOW)
     
def leftturn():
      GPIO.output(MOTORL1,GPIO.HIGH)
      GPIO.output(MOTORL2,GPIO.LOW)
      GPIO.output(MOTORR1,GPIO.LOW)
      GPIO.output(MOTORR2,GPIO.HIGH)

def stop():
      GPIO.output(MOTORL1,GPIO.LOW)
      GPIO.output(MOTORL2,GPIO.LOW)
      GPIO.output(MOTORR1,GPIO.LOW)
      GPIO.output(MOTORR2,GPIO.LOW)
     
#Image analysis
def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.cv.CV_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)      
    mask=cv2.dilate(mask,kern_dilate)    
    return mask

def find_blob(blob): #returns the red colored circle
    largest_contour=0
    cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area >largest_contour) :
            largest_contour=area
           
            cont_index=idx

                             
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])
       
    return r,largest_contour

def target_hist(frame):
    hsv_img=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
   
    hist=cv2.calcHist([hsv_img],[0],None,[50],[0,255])
    return hist

#CAMERA CAPTURE
camera = PiCamera()
camera.resolution = (160, 120)
camera.framerate = 16
rawCapture = PiRGBArray(camera, size=(160, 120))
 
time.sleep(0.001)
 

for image in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):  
      frame = image.array
      frame=cv2.flip(frame,1)
      global centre_x
      global centre_y
      centre_x=0.
      centre_y=0.
      hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask_red=segment_colour(frame)      #masking red the frame
      loct,area=find_blob(mask_red)
      x,y,w,h=loct
     
      #distance coming from front
      distanceC = sonar(TRIGGERF,ECHOF)
      #distance coming from right
      distanceR = sonar(TRIGGERR,ECHOR)
      #distance coming from left
      distanceL = sonar(TRIGGERL,ECHOL)
     
             
      if (w*h) < 10:
            found=0
      else:
            found=1
            simg2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            centre_x=x+((w)/2)
            centre_y=y+((h)/2)
            cv2.circle(frame,(int(centre_x),int(centre_y)),3,(0,110,255),-1)
            centre_x-=80
            centre_y=6--centre_y
            print centre_x,centre_y
      initial=400
      flag=0
               
      if(found==0):
            #if object not found, bot will rotate in previous seen direction
            if flag==0:
                  rightturn()
                  time.sleep(0.05)
            else:
                  leftturn()
                  time.sleep(0.05)
            stop()
            time.sleep(0.0125)
     
      elif(found==1):
            if(area<initial):
                  if(distanceC<10):
                        #if object is there, but obstacle in front of object, it avoids obstacle to reach object
                        if distanceR>=8:
                              rightturn()
                              time.sleep(0.00625)
                              stop()
                              time.sleep(0.0125)
                              forward()
                              time.sleep(0.00625)
                              stop()
                              time.sleep(0.0125)
                              #while found==0:
                              leftturn()
                              time.sleep(0.00625)
                        elif distanceL>=8:
                              leftturn()
                              time.sleep(0.00625)
                              stop()
                              time.sleep(0.0125)
                              forward()
                              time.sleep(0.00625)
                              stop()
                              time.sleep(0.0125)
                              rightturn()
                              time.sleep(0.00625)
                              stop()
                              time.sleep(0.0125)
                        else:
                              stop()
                              time.sleep(0.01)
                  else:
                       
                        forward()
                        time.sleep(0.00625)
            elif(area>=initial):
                  initial2=6700
                  if(area<initial2):
                        if(distanceC>10):
                              #move coordinates of object to center of camera's imaginary axis
                              if(centre_x<=-20 or centre_x>=20):
                                    if(centre_x<0):
                                          flag=0
                                          rightturn()
                                          time.sleep(0.025)
                                    elif(centre_x>0):
                                          flag=1
                                          leftturn()
                                          time.sleep(0.025)
                              forward()
                              time.sleep(0.00003125)
                              stop()
                              time.sleep(0.00625)
                        else:
                              stop()
                              time.sleep(0.01)

                  else:
                        #if it found the object and it is too close it stops
                        stop()
                        time.sleep(0.1)
      #cv2.imshow("draw",frame)    
      rawCapture.truncate(0)  # clear the stream in preparation for the next frame
         
      if(cv2.waitKey(1) & 0xff == ord('q')):
            break

GPIO.cleanup() #free all the GPIO pins



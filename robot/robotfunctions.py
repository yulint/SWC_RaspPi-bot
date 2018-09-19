import RPi.GPIO as GPIO
import numpy as np
import cv2 as cv
import random 

from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Set GPIO modes (numbering of pins: BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO pins
pinMotorRightF     = 7
pinMotorRightB     = 8
pinMotorLeftF      = 9
pinMotorLeftB      = 10
pinTrigger         = 17
pinEcho            = 18
pinServoRotation   = 22

# Set pin modes
GPIO.setup(pinServoRotation, GPIO.OUT)
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# Motors: 
# - B = Left
# - A = Right
GPIO.setup(pinMotorLeftF, GPIO.OUT) 
GPIO.setup(pinMotorLeftB, GPIO.OUT)  
GPIO.setup(pinMotorRightF, GPIO.OUT) 
GPIO.setup(pinMotorRightB, GPIO.OUT) 

# Set the GPIO to software PWM at 'Frequency' Hertz
Frequency=20
pwmMotorRightF = GPIO.PWM(pinMotorRightF, Frequency)
pwmMotorRightB = GPIO.PWM(pinMotorRightB, Frequency)
pwmMotorLeftF = GPIO.PWM(pinMotorLeftF, Frequency)
pwmMotorLeftB = GPIO.PWM(pinMotorLeftB, Frequency)

# Set idle states
pwmMotorRightF.start(0)
pwmMotorRightB.start(0)
pwmMotorLeftF.start(0)
pwmMotorLeftB.start(0)

GPIO.output(pinTrigger, 0)

# Start pinServoRotation as PWM with 50Hz and 0 duty
pwm = GPIO.PWM(pinServoRotation, 50)
pwm.start(0)


## External parameters ###################################################
#                                                                        #
# Some of them have to be optimized                                      #
#
# SpeedLeft = speedRight when dutyCycleLeft = 0.9 x dutyCycleRight       #
calibration = 0.9                                                        #
#
# Decide threshold distance (mm) to be considered as obstacle vs potential object
threshDist_obstacle = 10
# Smallest distance (mm) for which we can accept as potential object rather than obstacle
threshDist_object   = 100                                                #
#                                                                        #
##########################################################################

## FUNCTIONS

# Send a ultrasound pulse to check the distance to nearest object.
# Input: nothing
# Output: distance (in cm)
def distancePulse():
    # Send 10us pulse to trigger
    GPIO.output(pinTrigger, True)
    sleep(0.00001)
    GPIO.output(pinTrigger, False)

    # The start time is reset until the Echo pin is taken high (==1)
    StartTime = time()
    while GPIO.input(pinEcho) == 0:
        StartTime = time()

    StopTime = StartTime
    # Stop when the Echo pin is no longer high - the end time
    while GPIO.input(pinEcho) == 1:
        StopTime = time()

    # Return distance traveled by ultrasound pulse
    return (StopTime - StartTime) * 34326 / 2


# Sends a signal to the servo motor to set a given angle
# Input: angle (in degrees)
# Output: nothing
def rotateDistanceSensor(angle):
    # Duty is the ratio between times on and off
    # Our duty must be in the range 2-12 for 0-180 degrees
    duty = angle / 18 + 2
    
    # Change duty to correspond desired angle
    GPIO.output(pinServoRotation, True)
    pwm.ChangeDutyCycle(duty)
    
    # Maybe this is too much sleep time?
    sleep(1)
    
    # Back to no pulses
    GPIO.output(pinServoRotation, False)
    pwm.ChangeDutyCycle(0)


## Auxiliary motor functions          #############################
# They are not supposed to be used by the end user                # 
#                                                                 #
# Sends a signal to the on-off motors to go forwards or backwards.
# Input: nothing                                                  #
# Output: nothing                                                 #

def powerMotorRightForwards(speed):                               #
    DutyCycleRight = speed                                        #
    pwmMotorRightF.ChangeDutyCycle(DutyCycleRight)                #
    pwmMotorRightB.ChangeDutyCycle(0)                             #
                                                                  #
def powerMotorLeftForwards(speed):                                #
    DutyCycleLeft = speed * calibration                           #
    pwmMotorLeftF.ChangeDutyCycle(DutyCycleLeft)                  #
    pwmMotorLeftB.ChangeDutyCycle(0)                              #
                                                                  #
def powerMotorRightBackwards(speed):                              #
    DutyCycleRight = speed                                        #
    pwmMotorRightF.ChangeDutyCycle(0)                             #
    pwmMotorRightB.ChangeDutyCycle(DutyCycleRight)                #
                                                                  #
def powerMotorLeftBackwards(speed):                               #
    DutyCycleLeft = speed * calibration                           #
    pwmMotorLeftF.ChangeDutyCycle(0)                              #
    pwmMotorLeftB.ChangeDutyCycle(DutyCycleLeft)                  #
                                                                  #
def offAllMotors():                                               #
    pwmMotorRightF.ChangeDutyCycle(0)                             #
    pwmMotorRightB.ChangeDutyCycle(0)                             #
    pwmMotorLeftF.ChangeDutyCycle(0)                              #
    pwmMotorLeftB.ChangeDutyCycle(0)                              #
###################################################################


## User Motor functions

# Sends the robot forwards for some amount of time
# Input: duration (in s), speed (in duty cycles)
# Output: nothing
def goBackwards(speed, duration):

    # Turn on both motors backwards
    powerMotorLeftBackwards(speed)
    powerMotorRightBackwards(speed)

    # Duration time
    sleep(duration)

    # Clean the pins used so far
    offAllMotors()

# Sends the robot backwards for some amount of time
# Input: duration (in s)
# Output: nothing
def goForwards(speed, duration):
    # Turn on both motors forwards
    powerMotorLeftForwards(speed)
    powerMotorRightForwards(speed)

    # Duration time
    sleep(duration)

    # Clean the pins used so far
    offAllMotors()

# Sends the robot to turn left for some amount of time
# Input: duration (in s)
# Output: nothing
def turnLeft(speed, duration):
    # Turn only motors right forwards
    powerMotorRightForwards(speed)
    powerMotorLeftBackwards(speed)
    
    # Duration time
    sleep(duration)

    # Clean the pins used so far
    offAllMotors()

# Sends the robot to turn right for some amount of time
# Input: duration (in s)
# Output: nothing
def turnRight(speed, duration):
    # Turn only motors left forwards
    powerMotorRightBackwards(speed)
    powerMotorLeftForwards(speed)

    # Duration time
    sleep(duration)

    # Clean the pins used so far
    offAllMotors()

# Turns the robot to face 0, 45, 135 or 180degrees, left to right = 0 to 180
def turnAngle(angle):
    if angle == 0:
        turnLeft(40,0.6)

    if angle == 45:
        turnLeft(40,0.3)

    if angle == 135:
        turnRight(40,0.3)

    if angle == 180:
        turnRight(40,0.6)

    else
        print("We have an error, Houston!")


# Turns on camera
# Input: none
# Output: camera object in BGR mode 
def initializeCamera():
    # Initialize the camera
    camera = cv.VideoCapture(0)
    
    # Set camera resolution
    camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
   
    camera.set(cv.CAP_PROP_FPS, 1)
    
    # Warmup time for camera to pick optimal exposure/ gain values (maybe less?)
    sleep(1)

    # Fix exposure time, and gains (white balance cannot be fixed using openCV)
    optimizedExposure = int(camera.get(cv.CAP_PROP_EXPOSURE))
    camera.set(cv.CAP_PROP_EXPOSURE,optimizedExposure)
    
    return camera


# Receives a picture and process if it can find a blue object
# Input: picture (numpy array BGR)
# Output: pixel position of found object (-1 if not found)
def findBlueBalloon(image):
    # Our operations on the frame come here
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv = cv.medianBlur(hsv,5) #MAYBE WE SHOULD TRY GAUSSIAN BLUR HERE 

    # Define range of blue color in HSV
    # Royal blue's HSV:
    # - ColorHexa (225 o, 71.1, 88.2)
    # - Wikipedia (219 o, 100%, 40%)
    # - Color Hex (225 o, 71, 88)
    lower_blue = np.array([105,50,50])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image to get only blue colors
    blueMask = cv.inRange(hsv, lower_blue, upper_blue)

    # Refine mask
    blueMask = cv.medianBlur(hsv,5)
    kernel = np.ones((5,5),np.uint8)
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_OPEN, kernel)
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_CLOSE, kernel)

    # CODE TO WRITE HERE: find the blue balloon
    _, contours, hierarchy = cv.findContours(blueMask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    # METHOD ONE: choose largest contour
    if len(contours) > 0:
        largestContour = max(contours, key=cv.contourArea)
    
    # METHOD TWO: filter out contours that are too small, then check for roundness?
    sizeFilteredContours = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area > 1000 :
            sizeFilteredContours.append(contour)
    # code to check for roundness

    
    blueBalloon = largestContour #or whichever method we use
    
    # Find centroid of blue balloon
    M = cv.moments(blueBalloon)
    posX = int(M['m10']/M['m00'])
    posY = int(M['m01']/M['m00'])
   
    return posX, posY


# Receives a picture and the position of object then returns the angle to it
# Input: picture (numpy array BGR), (x, y) position in pixel
# Output: angle (2*pi*radians)
def getAngleFromImage(image, x, y):
    xcenter = image.shape[0] / 2

    rad = np.arctan((x - xcenter)/y)

    return 2*np.pi*rad


# Normalize intensity of images
# Input: picture (numpy array Gray)
# Output: picture (numpy array GRay)
def normalizeImage(img):
    rng = img.max() - img.min()

    return (img - img.min()) * 255 / rng


# Compare two images and check if they are "too" similar
# Input: 2 pictures (numpy array BGR)
# Output: True or False
def compareImages(oldimg, newimg):
    grayOldImg = normalizeImage(cv.cvtColor(oldimg, cv.COLOR_BGR2GRAY))
    grayNewImg = normalizeImage(cv.cvtColor(newimg, cv.COLOR_BGR2GRAY))

    # Calculate the difference and its norms
    diffimg = img1 - img2  # Elementwise for scipy arrays
    m_norm = sum(abs(diffimg))  # Manhattan norm
    z_norm = norm(diffimg.ravel(), 0)  # Zero norm

    # Return True if they are too similar 
    return (m_norm < 0.5 and z_norm < 0)


def compareDistanceSensor(previousD, currentD):
    difference = abs(previousD - currentD)
    
    return (difference < 1)
    

# MAIN FUNCTION OF THE ROBOT
# We should be able to run this and magic happens
def main():   
    camera = initializeCamera()
    # PreviousImg = 
    previousD = 0
           
    while True:
        # Capture frame-by-frame (for detecting blue balloons + checking if robot is stuck)
        _, currentImg  = camera.read()
        
        # Check distance straight ahead (for checking if robot is stuck)
        rotateDistanceSensor(90)
        currentD = distancePulse() 
        
        # Check whether robot is stuck after each movement
        stuck = compareImages(previousImg, currentImg) or compareDistanceSensor(previousD, currentD)
        if stuck == True:
            goBackwards(40,1)
            angle = (random.randint(0,6))*30
        
                    
        # LOOK FOR BLUE BALLOON
        # (x, y) are the position for the centroid of blue object
        x, y = findBlueBalloon(currentImg)

        # GO TOWARDS BLUE OBJECT, IF FOUND
        if (x,y) != (-1, -1):
            # Get angle of blue object 
            angle = 90 + getAngleFromImage(img, x, y)

            # Check for obstacles between robot and blue object
            rotateDistanceSensor(angle) # Turn the distance sensor towards blue object
            d = distancePulse()
            print("The distance is: ", d)
            
            # Walk straight to blue object if there are no obstacles in the way
            if d > threshDist_obstacle:
                turnAngle(angle)
                goForwards(40,1) # Need to optimise how far robot walks forwards

            if d <= threshDist_obstacle:
                (x,y) = (-1,-1)                 

        # EXPLORE IF NO BLUE OBJECTS
        # If no blue objects found, or obstacle in the way of blue object
        # Then use ultrasound distance sensor to search for potential objects and check for obstacles before turning
        # If no potential objects are found, pick a random direction 
        if (x, y) == (-1, -1): # Negative values: default for no object
            print("No object found!")

            listAnglesNoObstacles = []
            angle = -1
               
            for testAngle in range(0,180,45):  # Check through angles from 0 to 180, in 45degree increments 
                rotateDistanceSensor(testAngle)
                distance = distancePulse() 
                    
                if distance <= threshDist_object and distance > threshDist_obstacle: 
                    angle = testAngle
                    break
                
            # If no potential objects are found, pick a random angle amongst those without obstacles
                if distance > threshDist_obstacle and distance > threshDist_object:
                    listAnglesNoObstacles.append(testAngle)

            if angle == -1:
                angle = random.choice(listAnglesNoObstacles)        

            turnAngle(angle)
            goForwards(40,1)

        previousImg = currentImg
        previousD = currentD
        
    camera.release()
            

# Testing

goForwards(40, 1)

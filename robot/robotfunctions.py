import RPi.GPIO as GPIO
import numpy as np
import cv2 as cv
import random 

from time import sleep, time

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
# Decide threshold distance (cm) to be considered as obstacle vs potential object
threshDist_obstacle = 20
# largest distance (mm) for which we can accept as potential object that is close enough
threshDist_object   = 300                                                #
# the threshold to consider an object round enough to be a balloon
roundnessThreshold  = 0.9                                                #
# how much the roundness should be important with respect to the size of blue object
powerRoundness      = 2                                                  #
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

    elif angle == 45:
        turnLeft(40,0.3)

    elif angle == 135:
        turnRight(40,0.3)

    elif angle == 180:
        turnRight(40,0.6)

    elif angle == 90:
        pass

    else:
        print("We have an error, Houston!")
        print("somehow ", angle)


# Turns on camera
# Input: none
# Output: camera object in BGR mode 
def initializeCamera():
    # Initialize the camera
    camera = cv.VideoCapture(0)

    # Set camera resolution
    camera.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

    # set the buffer to 1, otherwise the camera.grab() will give old frames
    camera.set(cv.CAP_PROP_BUFFERSIZE, 1)
               
    # camera.set(cv.CAP_PROP_FPS, 30  )

    # Warmup time for camera to pick optimal exposure/ gain values (maybe less?)
    sleep(1)

    # Fix exposure time, and gains (white balance cannot be fixed using openCV)
    # optimizedExposure = int(camera.get(cv.CAP_PROP_EXPOSURE))
    # camera.set(cv.CAP_PROP_EXPOSURE,optimizedExposure)
    _, testimg = camera.read()
    if testimg is None:
        print("None!")
    
    return camera


# Receives a picture and process if it can find a blue object
# Input: picture (numpy array BGR)
# Output: pixel position of found object (-1 if not found) and best contour's area
def findBlueBalloon(image, i):
    # default x,y values if no objects are found
    (posX, posY) = (-1,-1)

    # Our operations on the frame come here
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv = cv.GaussianBlur(hsv,(7,7),0)
    
    # Define range of blue color in HSV
    lower_blue = np.array([75,75,50])
    upper_blue = np.array([130,255,255])

    # Threshold the HSV image to get only blue colors
    blueMask = cv.inRange(hsv, lower_blue, upper_blue)

    # Refine mask
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (12,12))
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_CLOSE, kernel)
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_OPEN, kernel)
    
    # CODE TO WRITE HERE: find the blue balloon
    _, contours, hierarchy = cv.findContours(blueMask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # set a inexistent balloon as default
    bestBalloon = (0, 0), (0, 0), 0
    # set inexistent balloon`s parameters as well
    # - roundmess measures how much it is similar to an ellipse
    # - goodness = roundness^powerRoundness * areaContour
    balloonRoundness = 0.
    balloonGoodness  = 0.
    bestcontour = 0.
    
    for contour in contours:
        # find best ellipse describe the contour
        ellipse = cv.fitEllipse(contour)
        # split the ellipse's parameters
        (xel, yel), (r1el, r2el), rot = ellipse
    
        areacontour = cv.contourArea(contour)
        areaellipse = np.pi * r1el * r2el / 4 # axis, not semi-axis

        roundness = 1 - np.abs(areacontour - areaellipse) / max(areacontour, areaellipse)
        goodness  = roundness**powerRoundness * areacontour

        # save IF (bigger and better roundness) AND (round enough)
        if goodness > balloonGoodness and roundness > roundnessThreshold:
            # save best values so far for next iteration
            balloonRoundness = roundness
            balloonGoodness = goodness
            bestBalloon = ellipse
            bestcontour = areacontour

            # save position of centroid to be returned by the function
            posX = xel
            posY = yel

    if balloonGoodness != 0:
        cv.ellipse(image, bestBalloon,(0,0,255), 2)
        cv.imwrite('pics/pic_{}_balloon.jpg'.format(i), image)

    return posX, posY, bestcontour


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
    grayOldImg = cv.cvtColor(oldimg, cv.COLOR_BGR2GRAY) # normalizeImage
    grayNewImg = cv.cvtColor(newimg, cv.COLOR_BGR2GRAY)

    grayOldImg_float = np.float32(grayOldImg)
    grayNewImg_float = np.float32(grayNewImg)
    
    # Calculate the difference and its norms
    diffimg = np.abs(grayOldImg_float - grayNewImg_float) # Elementwise for scipy arrays
    absdiffmean = np.mean(diffimg)  # Manhattan norm
    diffpixels = np.count_nonzero(diffimg > 30)/(640*480.) # Zero norm after threshold

    print("mean of abs diff: ", absdiffmean)
    print("fraction diff: ", diffpixels)

    print(diffimg)

    # Return True if they are too similar (robot stuck)
    return (absdiffmean < 12 and diffpixels < 0.05)


def compareDistanceSensor(previousD, currentD):
    difference = abs(previousD - currentD)
    
    return (difference < 10 and difference > 0) #expect that robot should move at least 1 cm each yimr 
    

def roundTo45(n):
    return 45*((n + 22.5) // 45)

# MAIN FUNCTION OF THE ROBOT
# We should be able to run this and magic happens
def main():   
    camera = initializeCamera()
    previousImg = camera.read()
    previousD = -1

    print(0)
    cv.imwrite('pics/pic_0.jpg', previousImg)
    
    goForwards(40, 1)

    i = 0
    while True:
        # Capture frame-by-frame (for detecting blue balloons + checking if robot is stuck)
        _, currentImg  = camera.read()
        
        i += 1
        print(i)
        cv.imwrite('pics/pic_{}.jpg'.format(i), currentImg)
        
        # Check distance straight ahead (for checking if robot is stuck)
        rotateDistanceSensor(90)
        currentD = distancePulse() 
        print("distance straight ahead:", currentD)

        # LOOK FOR BLUE BALLOON
        # (x, y) are the position for the centroid of blue object
        x, y, area = findBlueBalloon(currentImg, i)
        print('blue object!\n x: {} y: {} area:{}'.format(x, y, area))
        
        # Check whether robot is stuck after each movement
        imagestuck = compareImages(previousImg, currentImg)
        distancestuck = compareDistanceSensor(previousD, currentD)

        if imagestuck or (distancestuck and area < 70000):
            goBackwards(40,1)
            angle = (random.randint(0,6))*30
            turnAngle(roundTo45(angle))
            goForwards(40, 1)
            print("stuck! turn ",roundTo45(angle))
            print("image: ", imagestuck, " distance: ", distancestuck)
            # bad coding! fix this..
            previousImg = currentImg
            previousD = currentD
            sleep(5)
            continue
                    
        # GO TOWARDS BLUE OBJECT, IF FOUND
        if (x,y) != (-1, -1):
            # Get angle of blue object 
            angle = 90 + getAngleFromImage(currentImg, x, y)

            # Check for obstacles between robot and blue object
            rotateDistanceSensor(angle) # Turn the distance sensor towards blue object
            d = distancePulse()
                        
            # Walk straight to blue object if there are no obstacles in the way
            if (d <= threshDist_obstacle and area < 70000):
                (x,y) = (-1,-1)
                print("object found but obstacle ahead at distance ", d)
            else:
                turnAngle(roundTo45(angle))
                goForwards(40,1) # Need to optimise how far robot walks forwards
                print("move towards blue object ", d)
                print("blue object angle ", roundTo45(angle))


        # EXPLORE IF NO BLUE OBJECTS
        # If no blue objects found, or obstacle in the way of blue object
        # Then use ultrasound distance sensor to search for potential objects and check for obstacles before turning
        # If no potential objects are found, pick a random direction 
        if (x, y) == (-1, -1): # Negative values: default for no object
            print("No object found, explore!")

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

            turnAngle(roundTo45(angle))
            goForwards(50,1) #move faster when exploring

        previousImg = currentImg
        previousD = currentD

        sleep(5)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
    camera.release()
            

# Testing

main() 

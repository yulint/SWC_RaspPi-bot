import RPi.GPIO as GPIO
import numpy as np
import cv2 as cv
import random 


from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

# set GPIO modes (numbering of pins: BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# define GPIO pins
pinMotorRightF     = 7
pinMotorRightB     = 8
pinMotorLeftF      = 9
pinMotorLeftB      = 10
pinTrigger         = 17
pinEcho            = 18
pinServoRotation   = 22

# set pin modes
GPIO.setup(pinTrigger, GPIO.OUT)
GPIO.setup(pinServoRotation, GPIO.OUT)
GPIO.setup(pinEcho, GPIO.IN)

# motors: 
# - B = Left
# - A = Right
GPIO.setup(pinMotorLeftF, GPIO.OUT) 
GPIO.setup(pinMotorLeftB, GPIO.OUT)  
GPIO.setup(pinMotorRightF, GPIO.OUT) 
GPIO.setup(pinMotorRightB, GPIO.OUT) 
# Set the GPIO to software PWM at 'Frequency' Hertz
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

    # return distance traveled by ultrasound pulse
    return (StopTime - StartTime) * 34326 / 2


# Sends a signal to the servo motor to set a given angle
# Input: angle (in degrees)
# Output: nothing
def setAngle(angle):
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
calibration = 0.9 #Speed of 2 motors are same when dutyCycleLeft is 0.9 x dutyCycleRight

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
def stopMotors():                                                 #
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

    # turn on both motors backwards
    powerMotorLeftBackwards(speed)
    powerMotorRightBackwards(speed)

    # duration time
    sleep(duration)

    # clean the pins used so far
    offAllMotors()

# Sends the robot backwards for some amount of time
# Input: duration (in s)
# Output: nothing
def goForwards(speed, duration):
    # turn on both motors forwards
    powerMotorLeftForwards(speed)
    powerMotorRightForwards(speed)

    # duration time
    sleep(duration)

    # clean the pins used so far
    offAllMotors()

# Sends the robot to turn left for some amount of time
# Input: duration (in s)
# Output: nothing
def turnLeft(speed, duration):
    # turn only motors right forwards
    powerMotorRightForwards(speed)
    powerMotorLeftBackwards(speed)
    
    # duration time
    sleep(duration)

    # clean the pins used so far
    offAllMotors()

# Sends the robot to turn right for some amount of time
# Input: duration (in s)
# Output: nothing
def turnRight(speed, duration):
    # turn only motors left forwards
    powerMotorRightBackwards(speed)
    powerMotorLeftForwards(speed)

    # duration time
    sleep(duration)

    # clean the pins used so far
    offAllMotors()

# Turns the robot to face 0, 45, 135 or 180degrees, left to right = 0 to 180
def turnAngle(angle)
    if angle == 0:
        turnLeft(40,0.6)

    if angle == 45:
        turnLeft(40,0.3)

    if angle == 135:
        turnRight(40,0.3)

    if angle == 180
        turnRight(40,0.6)


# Grabs a picture from PiCamera
# Input: nothing
# Output: picture (numpy array BGR)
def grabPicture():
    # Initialize the camera, set resolution, fix rotation
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.rotation = 180

    # Camera warmup time (maybe less?)
    sleep(1)

    # Fix exposure time, white balance and gains
    camera.shutter_speed = camera.exposure_speed
    g = camera.awb_gains
    camera.awb_mode = "off"
    camera.awb_gains = g

    # Grab a reference to the raw camera capture
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # Capture frames from the camera at rawCapture
    camera.capture(rawCapture, format="bgr")

    # Grab the NumPy array representing the raw image
    image = rawCapture.array

    # Clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # Everything done, release the capture
    # Maybe we should save it before ending
    camera.close()

    return image


# Receives a picture and process if it can find a blue object
# Input: picture (numpy array BGR)
# Output: pixel position of found object (-1 if not found)
def findBlueBalloon(image):
    # Our operations on the frame come here
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    hsv = cv.medianBlur(hsv,5)

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
    kernel = np.ones((5,5),np.uint8)
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_OPEN, kernel)
    blueMask = cv.morphologyEx(blueMask, cv.MORPH_CLOSE, kernel)

    # Bitwise-AND mask and original image
    blueImage = cv.bitwise_and(image, image, mask=blueMask)

    # CODE TO WRITE HERE: find the blue balloon

    # Find centroid of object
    M = cv.moments(blueMask)
    area = M['m00']

    # If there are no objects, return -1
    posX = -1
    posY = -1
    # If the area is too small maybe that's not an object we want
    if area > 10000:
        posX = int(M['m10']/M['m00'])
        posY = int(M['m01']/M['m00'])

    return posX, posY


# Receives a picture and the position of object then returns the angle to it
# Input: picture (numpy array BGR), (x, y) position in pixel
# Output: angle (2*pi*radians)
def getAngleFromImage(image, x, y):
    xcenter = image.shape[0] / 2

    rad = np.arctan((np.abs(xb - xp))/yb)

    return 2*np.pi*rad

# Normalize intensity of images
# Input: picture (numpy array Gray)
# Output: picture (numpy array GRay)
def normalize(img):
    rng = img.max() - img.min()

    return (img - img.min()) * 255 / rng

# Compare two images and check if they are "too" different
# Input: 2 pictures (numpy array BGR)
# Output: True or False
def compareImages(oldimg, newimg):
    grayOldImg = normalize(cv.cvtColor(oldimg, cv.COLOR_BGR2GRAY))
    grayNewImg = normalize(cv.cvtColor(newimg, cv.COLOR_BGR2GRAY))

    # calculate the difference and its norms
    diffimg = img1 - img2  # elementwise for scipy arrays
    m_norm = sum(abs(diffimg))  # Manhattan norm
    z_norm = norm(diffimg.ravel(), 0)  # Zero norm

    # return True if they are different enough
    return (m_norm > 0.5 and z_norm > 0)



# MAIN FUNCTION OF THE ROBOT
# We should be able to run this and magic happens
def main():
    while True:
        img  = grabPicture()

        # (x, y) are the position for the centroid of blue object
        x, y = findBlueBalloon(img)

        # found something blue!
        if (x,y) != (-1, -1):
            # get angle of blue object
            angle = getAngleFromImage(img, x, y)

            # turn the distance sensor towards blue object
            setAngle(angle)
            # measure the distance towards blue object
            d = distancePulse()
            print("The distance is: ", d)

            # now walk to blu object

        # if no blue objects found
        if (x, y) == (-1, -1): # negative values: default for no object
            print("No object found!")

            # use ultrasound distance sensor to search for potential objects and check for obstacles before turning, overriding the random angle if potential objects are found
            threshDist_object = 100 #smallest distance (mm) for which we can accept as potential object rather than obstacle; needs to be optimized
            threshDist_obstacle = 10

            listAnglesNoObstacles = []
            angle = -1
               
            for testAngle in range(0,180,45):  #check through angles from 0 to 180, in 45degree increments 
                setAngle(testAngle)
                distance = distancePulse() 
                    
                if distance <= threshDist_object and distance > threshDist_obstacle: 
                    angle = testAngle
                    break
                
            # if no potential objects are found, pick a random angle amongst those without obstacles
                if distance > threshDist_obstacle and distance > threshDist_object:
                    listAnglesNoObstacles.append(testAngle)
            if angle == -1:
                angle = random.choice(listAnglesNoObstacles)        

            turnAngle(angle)   


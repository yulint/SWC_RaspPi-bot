def functeste():
    print(2)


# Setup the Pins and all
# Input: nothing
# Output: nothing
def setupRobot():
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
    # motors: 
    # - B = Left
    # - A = Right
    GPIO.setup(pinTrigger, GPIO.OUT)
    GPIO.setup(pinServoRotation, GPIO.OUT)
    GPIO.setup(pinMotorLeftF, GPIO.OUT) 
    GPIO.setup(pinMotorLeftB, GPIO.OUT)  
    GPIO.setup(pinMotorRightF, GPIO.OUT) 
    GPIO.setup(pinMotorRightB, GPIO.OUT) 
               
    GPIO.setup(pinEcho, GPIO.IN)

    # Set idle states
    GPIO.output(pinMotorLeftF,0)
    GPIO.output(pinMotorLeftB,0)
    GPIO.output(pinMotorRightF,0)
    GPIO.output(pinMotorRightB,0)

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
def powerMotorRightForwards():                                    #
    GPIO.output(pinMotorRightF,1)                                 #
    GPIO.output(pinMotorRightB,0)                                 #
                                                                  #
def powerMotorLeftForwards():                                     #
    GPIO.output(pinMotorLeftF,1)                                  #
    GPIO.output(pinMotorLeftB,0)                                  #
                                                                  #
def powerMotorRightBackwards():                                   #
    GPIO.output(pinMotorRightF,0)                                 #
    GPIO.output(pinMotorRightB,1)                                 #
                                                                  #
def powerMotorLeftBackwards():                                    #
    GPIO.output(pinMotorLeftF,0)                                  #
    GPIO.output(pinMotorLeftB,1)                                  #
###################################################################


## User Motor functions

# Sends the robot forwards for some amount of time
# Input: duration (in s)
# Output: nothing
def goBackwards(duration):
    # turn on both motors backwards
    powerMotorLeftBackwards()
    powerMotorRightBackwards()

    # duration time
    sleep(duration)

    # clean the pins used so far
    GPIO.cleanup()

# Sends the robot backwards for some amount of time
# Input: duration (in s)
# Output: nothing
def goForwards(duration):
    # turn on both motors forwards
    powerMotorLeftForwards()
    powerMotorRightForwards()

    # duration time
    sleep(duration)

    # clean the pins used so far
    GPIO.cleanup()

# Sends the robot to turn left for some amount of time
# Input: duration (in s)
# Output: nothing
def turnLeft(duration):
    # turn only motors right forwards
    powerMotorRightForwards()

    # duration time
    sleep(duration)

    # clean the pins used so far
    GPIO.cleanup()

# Sends the robot to turn right for some amount of time
# Input: duration (in s)
# Output: nothing
def turnRight(duration):
    # turn only motors left forwards
    powerMotorLeftBackwards()

    # duration time
    sleep(duration)

    # clean the pins used so far
    GPIO.cleanup()

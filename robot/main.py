import RPi.GPIO as GPIO
from time import sleep, time
from robotfunctions import *

setup()

## testing

d = distancePulse()
print(d)

setAngle(90)

motorRight(1.7, 1)               

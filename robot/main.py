import RPi.GPIO as GPIO
import numpy as np
import cv2 as cv

from time import sleep, time
from picamera import PiCamera
from picamera.array import PiRGBArray

from robotfunctions import *

setup()

## testing

d = distancePulse()
print(d)

setAngle(90)

motorRight(1.7, 1)               

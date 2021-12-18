#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    InfraredSensor,
    UltrasonicSensor,
    GyroSensor,
)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font

EV3 = EV3Brick()
rightColorSensor = ColorSensor(Port.S3)
leftColorSensor = ColorSensor(Port.S1)
# Initiate the motors
leftMotor = Motor(Port.B, Direction.CLOCKWISE)
rightMotor = Motor(Port.C, Direction.CLOCKWISE)
wheelDiameter = 62.5
axleTrack = 102.5
verticalMotor = Motor(Port.D, Direction.CLOCKWISE)
horizonalMotor = Motor(Port.A, Direction.CLOCKWISE)

# Initiate the drive base:
robot = DriveBase(
    leftMotor, rightMotor, wheel_diameter=wheelDiameter, axle_track=axleTrack
)

# set up the gyro sensor
gyroSensor = GyroSensor(Port.S2, Direction.CLOCKWISE)

kpValue = 1.1
kiValue = 0
kdValue = 1

# read the calibration data
CS_FileName = "/home/robot/CargoConnect21/CalibrationData.txt"
CS_LEFT = "left"
CS_RIGHT = "right"
CS_WHITE = "white"
CS_BLACK = "black"

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
from pybricks.media.ev3dev import SoundFile, ImageFile

import sys, time

sys.path.append(sys.path[0] + "/..")

from helpers import constants


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# set up the gyro sensor


turnAmount = 90

turnSpeed = 200
startingAngle = constants.gyroSensor.angle()
currentAngle = startingAngle
currentTurnAngle = currentAngle - startingAngle
start_time = time.time()
if turnAmount > 0:
    while currentTurnAngle <= turnAmount:
        currentAngle = constants.gyroSensor.angle()
        currentTurnAngle = currentAngle - startingAngle
        print("The current Degree of the turn is ", currentTurnAngle)
        constants.leftMotor.run(turnSpeed)
        constants.rightMotor.run(-turnSpeed)
else:
    while currentTurnAngle >= turnAmount:
        currentAngle = constants.gyroSensor.angle()
        currentTurnAngle = currentAngle - startingAngle
        print("The current Degree of the turn is ", currentTurnAngle)
        constants.leftMotor.run(-turnSpeed)
        constants.rightMotor.run(turnSpeed)

constants.leftMotor.brake()
constants.rightMotor.brake()
elapsed_time = time.time() - start_time
wait(1000)
print(
    "The current Degree of the turn is ",
    currentTurnAngle,
    " The elapsed time is ",
    elapsed_time,
    "The current gyro angle is ",
    constants.gyroSensor.angle(),
)

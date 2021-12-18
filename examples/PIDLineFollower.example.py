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

import sys

sys.path.append(sys.path[0] + "/..")

from helpers import PIDLineFollower, ColorSensorHelper, constants


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


targetLRI = constants.targetLRIForRightColorSensor
targetCheckPointLRI = constants.blackLRIForLeftColorSensor

constants.robot.straight(100)

targetLeftMotorAngle = 2000
robotSpeed = 75

PIDLineFollower.followLinePidSimplified(
    constants.rightColorSensor,
    constants.leftColorSensor,
    False,
    150,
    targetLRI,
    targetCheckPointLRI,
    constants.kpValue,
    constants.kiValue,
    constants.kdValue,
    targetLeftMotorAngle,
    False,
)

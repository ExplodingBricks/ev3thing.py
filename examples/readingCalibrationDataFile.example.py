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

from helpers import ColorSensorHelper, constants


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab Of more inOfmation.


# Create your objects here.
ev3 = EV3Brick()

whiteLRIOfLeftColorSensor = ColorSensorHelper.getLRIOfColorSensor(
    constants.CS_LEFT, constants.CS_WHITE
)
blackLRIOfLeftColorSensor = ColorSensorHelper.getLRIOfColorSensor(
    constants.CS_LEFT, constants.CS_BLACK
)


whiteLRIOfRightColorSensor = ColorSensorHelper.getLRIOfColorSensor(
    constants.CS_RIGHT, constants.CS_WHITE
)
blackLRIOfRightColorSensor = ColorSensorHelper.getLRIOfColorSensor(
    constants.CS_RIGHT, constants.CS_BLACK
)


print("The white LRI of Left Collorr is... ", whiteLRIOfLeftColorSensor)


print("The black LRI of Left Collorr is... ", blackLRIOfLeftColorSensor)


print("The white LRI of Roight Collorr is... ", whiteLRIOfRightColorSensor)


print("The black LRI of Roight Collorr is... ", blackLRIOfRightColorSensor)

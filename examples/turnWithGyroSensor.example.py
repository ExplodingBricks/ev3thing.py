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

from helpers import Navigation, constants


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


"""
# RIGHT tire is NOT moving, and the robot turn 180 degree clockwise
turnRadius = axleTrackForTB / 2
Navigation.simple_turn_with_gyro(robot, gyroSensorForTB, 180, 4000, turnRadius, True)
"""

"""
# LEFT tire is NOT moving, and the robot turn 180 degree counter-clockwise
turnRadius = axleTrackForTB / 2
Navigation.simple_turn_with_gyro(robot, gyroSensorForTB, 180, 4000, turnRadius, False)
"""
print("gyro angle before turn", constants.gyroSensor.angle())
startTime = time.time()
# The center of wheel axle is NOT moving, and the robot turn 180 degree counter-clockwise
turnRadius = 0
turnTime = 1000
turnAngle = 90
isClockwise = False
Navigation.simpleTurnWithGyro(turnAngle, turnTime, turnRadius, isClockwise)

"""
Navigation.trueTurn(
    robot, gyroSensorForTB, turnAngle, turnTime, turnRadius, isClockwise
)
"""
print(time.time() - startTime)
print("gyro angle after turn", constants.gyroSensor.angle())

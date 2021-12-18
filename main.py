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
import time, sys

sys.path.append(sys.path[0] + "/")
from helpers import ColorSensorHelper, constants
from runs import Run #Import your runs here

fileName = "/home/robot/REPO_NAME/CalibrationData.txt"


def calibrate():
    ColorSensorHelper.calibrateAllColorSensors(fileName)


def printCalibration():
    whiteLRIOfLeftColorSensor = ColorSensorHelper.getLRIOfColorSensor(
        constants.CS_LEFT, constants.CS_WHITE
    )
    blackLRIOfLeftColorSensor = ColorSensorHelper.getLRIOfColorSensor(
        constants.CS_LEFT, constants.CS_BLACK
    )
    left = (
        "Left:" + str(whiteLRIOfLeftColorSensor) + ", " + str(blackLRIOfLeftColorSensor)
    )

    whiteLRIOfRightColorSensor = ColorSensorHelper.getLRIOfColorSensor(
        constants.CS_RIGHT, constants.CS_WHITE
    )
    blackLRIOfRightColorSensor = ColorSensorHelper.getLRIOfColorSensor(
        constants.CS_RIGHT, constants.CS_BLACK
    )
    right = (
        "Right:"
        + str(whiteLRIOfRightColorSensor)
        + ", "
        + str(blackLRIOfRightColorSensor)
    )
    constants.EV3.screen.draw_text(10, 30, left)
    constants.EV3.screen.draw_text(10, 55, right)

    while True:
        if ev3.buttons.pressed():
            ev3.light.on(Color.RED)
            break
        wait(500)


Metadata = [
    ("Calibrate", "", calibrate),
    ("Print Calibration", "", printCalibration),
]
Setup = []
big_font = Font(size=30, bold=True)
small_font = Font(size=15, bold=False)


def displayMenuItem(menuItem: str, currentEV3: EV3Brick, num):
    currentEV3.screen.clear()
    currentEV3.screen.set_font(big_font)
    currentEV3.screen.draw_text(10, 50, menuItem)

    currentEV3.screen.set_font(small_font)
    currentEV3.screen.draw_text(10, 10, Metadata[num][0])
    currentEV3.screen.draw_text(10, 25, Metadata[num][1])


def getMenuName(menuNumber: int):

    # these lines choose which menu to display

    return Metadata[menuNumber][0]


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.

totalruns = len(Metadata) - 1
global currentMenuNumber
currentMenuNumber = 0
autoscroll = True


def limitcheck(number, totalruns):
    num = number
    if number < 0:
        num = totalruns
    elif number > totalruns:
        num = 0
    return num


def updateScreen(currentMenuNumber):
    displayMenuItem(getMenuName(currentMenuNumber), ev3, currentMenuNumber)


updateScreen(currentMenuNumber)

while True:

    # Respond to the Brick Button press.

    # Check whether Up Button is pressed, and increase the steps
    # variable by 1 if it is.
    if Button.DOWN in ev3.buttons.pressed():
        currentMenuNumber = limitcheck(currentMenuNumber - 1, totalruns)
        updateScreen(currentMenuNumber)

    # Check whether Down Button is pressed, and decrease the steps
    # variable by 1 if it is.
    if Button.UP in ev3.buttons.pressed():
        currentMenuNumber = limitcheck(currentMenuNumber + 1, totalruns)
        updateScreen(currentMenuNumber)

    if Button.CENTER in ev3.buttons.pressed():
        print("Running item", currentMenuNumber)
        ev3.light.on(Color.RED)
        # wait for half an second so the operator have enough time to press button before the robot rushes away
        wait(500)
        Metadata[currentMenuNumber][2]()
        if len(Metadata[currentMenuNumber]) > 3:
            Setup[Metadata[currentMenuNumber][3]]()
        ev3.light.off()

        # advance the menu item to the next one
        currentMenuNumber = limitcheck(currentMenuNumber + 1, totalruns)
        updateScreen(currentMenuNumber)

    wait(250)

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
from helpers import constants


def getLightReflection(currentColorSensor: ColorSensor) -> int:
    """Gets the LRI reading from the COLOR SENSOR.

    Args:
        currentColorSensor (ColorSensor): The COLOR SENSOR to get the LRI reading from.

    Returns:
        int: The LRI reading from the COLOR SENSOR.
    """
    return currentColorSensor.reflection()


def getAllColorCalibrationData(calibrationDataFileName: str) -> tuple:
    """Gets all color calibration data and returns it as a tuple.

    Args:
        calibrationDataFileName (str): The name of the CSV file to get the calibration data from.

    Returns:
        tuple: The tuple containing the color calibration data.
    """
    calibrationData = []

    # Opens Calibration Data File in READ-ONLY mode
    with open(calibrationDataFileName, "r") as calDatafile:
        calibrationDataList = calDatafile.readlines()
        # For each line in the data file...
        for row in calibrationDataList[0:]:
            # ...it extracts the values from the CSV data
            colorSensor, whiteLRI, blackLRI = row.strip().split(",")

            calData_dict = (colorSensor, whiteLRI, blackLRI)

            # Creates a tuple from the csv data and adds it to the data list
            calibrationData.append(calData_dict)

    return calibrationData


def getLRIOfColorSensor(side: str, color: str) -> int:
    """Returns the reading of LRI
    Args:
        calData ([]): Calibration data to be read
        side (str): `right` or `left`
        color (str): `black` or `white`
    Returns:
        [int]: The stored LRI
    """
    LRI = 90
    calData = getAllColorCalibrationData(constants.CS_FileName)
    if color == "black":
        pos = 2
    elif color == "white":
        pos = 1
    for currentCalTuple in calData:
        currentColorSensorPosition = currentCalTuple[0]
        loweredCurrentColorSensorPosition = currentColorSensorPosition.lower()
        if side in loweredCurrentColorSensorPosition:
            LRI = int(currentCalTuple[pos])
            break

    return LRI


def calibrateColorSensor(
    ev3: EV3Brick,
    currentColorSensor: ColorSensor,
    leftOrRight: str,
    calFileName: str,
):
    """Calibrates the selected COLOR SENSOR by getting LRI readings from both White and Black areas and stores it in the CSV Data file.

    Args:
        currentColorSensor (ColorSensor): The COLOR SENSOR to calibrate.
        leftOrRight (str): `left` or `right`
        calFileName (str): The name of the CSV file to store the calibration data.
    """
    # Gives text prompt to put the selected COLOR SENSOR over a White AREA.
    bigFont = Font(size=20, bold=True)
    ev3.screen.clear()
    ev3.screen.set_font(bigFont)
    ev3.screen.draw_text(0, 30, "Place " + leftOrRight + " color")
    ev3.screen.draw_text(0, 50, "sensor over WHITE")
    ev3.screen.draw_text(0, 70, "Press any button..")
    # Waits till any buttons on the brick are pressed
    while True:
        if ev3.buttons.pressed():
            ev3.light.on(Color.RED)
            break
        wait(500)
    wait(2000)
    # Once the button is pressed, it takes the current LRI reading and stores it as the
    # LRI reading for the white area. Reduce the value by 2 to accommodate the effect of ambient lights
    whiteReflection = currentColorSensor.reflection() - 2
    ev3.light.off()

    # Gives a text prompt to put the selected COLOR SENSOR over a Black AREA
    ev3.screen.clear()
    ev3.screen.set_font(bigFont)
    ev3.screen.draw_text(0, 30, "Place " + leftOrRight + " color")
    ev3.screen.draw_text(0, 50, "sensor over BLACK")
    ev3.screen.draw_text(0, 70, "Press any button..")

    wait(1500)
    # Waits until a button is pressed on the brick
    while True:
        if ev3.buttons.pressed():
            print("button pressed whilse ove black area.")
            ev3.light.on(Color.RED)
            break
        wait(500)

    wait(2000)

    # Once the button on the brick is pressed, it stores the current LRI reading as the
    # LRI reading for a black area. Increase value by 2 to accommodate the effect of ambient lights
    blackReflection = currentColorSensor.reflection() + 2

    # Opens the Calibration CSV data file in append mode, and the file will be
    # automatically closed once the writing is complete
    with open(calFileName, "a") as colorSensorCalibrationDataFile:

        # Adds the new LRI readings to the Calibration CSV data file
        colorSensorCalibrationDataFile.write(
            leftOrRight + "," + str(whiteReflection) + "," + str(blackReflection) + "\n"
        )

    # Tells you on the Ev3 brick "CLABRATION SUCCESSFULL!!!!"
    ev3.screen.clear()
    ev3.screen.draw_text(0, 40, leftOrRight + " done")
    ev3.light.off()


def calibrateAllColorSensors(fileName: str):
    """Takes the LRI readings from both COLOR SENSORS, and stores them in a CSV file.

    Args:
        fileName (str): The name of the CSV file to store the calibration data.

    """
    # Erases previous content and makes the file blank
    with open(fileName, "w") as currCalFileName:
        pass

    # Takes the LRI readings from over the WHITE and BLACK areas by the Right COLOR SENSOR
    calibrateColorSensor(constants.EV3, constants.rightColorSensor, "right", fileName)

    wait(1500)

    # Takes the LRI readings from over the WHITE and BLACK areas by the Left COLOR SENSOR
    calibrateColorSensor(constants.EV3, constants.leftColorSensor, "left", fileName)


def getTargetLRIForLeftCS():
    leftBlackLRI = getLRIOfColorSensor("left", "black")
    leftWhiteLRI = getLRIOfColorSensor("left", "white")
    targetLRIForLeftCS = (leftBlackLRI + leftWhiteLRI) / 2
    return targetLRIForLeftCS


def getTargetLRIForRightCS():
    rightBlackLRI = getLRIOfColorSensor("right", "black")
    rightWhiteLRI = getLRIOfColorSensor("right", "white")
    targetLRIForRightCS = (rightBlackLRI + rightWhiteLRI) / 2
    return targetLRIForRightCS


def getWhiteLRIForLeftCS():
    leftWhiteLRI = getLRIOfColorSensor("left", "white")
    return leftWhiteLRI


def getWhiteLRIForRightCS():
    rightWhiteLRI = getLRIOfColorSensor("right", "white")
    return rightWhiteLRI

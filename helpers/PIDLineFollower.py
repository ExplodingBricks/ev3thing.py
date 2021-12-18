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
from helpers import constants
import math


def followLinePidForCertainMotorRotatingDegree(
    lineFollowingColorSensor: ColorSensor,
    followLeftEdge: Boolean,
    targetLineFollowingLRI: int,
    currentDriveSpeed: int,
    targetMotorAngle: int,
    kpValue: float,
    kiValue: float,
    kdValue: float,
):
    """Follows the line using the PID controller for a certain number of degrees.

    Args:
        lineFollowingColorSensor (ColorSensor): The color sensor that is used to follow the line.
        followLeftEdge (Boolean):  If true, the robot follows the left edge of the line.
        targetLineFollowingLRI (int): The target light reflection index of the line.
        currentDriveSpeed (int): The current drive speed of the robot.
        targetMotorAngle (int): The target motor angle of the robot.
        kpValue (float): The value of the proportional term of the PID controller.
        kiValue (float): The value of the integral term of the PID controller.
        kdValue (float): The value of the derivative term of the PID controller.
    """
    currentcheckpointColorSensor = lineFollowingColorSensor
    ignoreCheckPoint = True

    followLinePidSimplified(
        lineFollowingColorSensor,
        currentcheckpointColorSensor,
        followLeftEdge,
        currentDriveSpeed,
        targetLineFollowingLRI,
        targetLineFollowingLRI,
        kpValue,
        kiValue,
        kdValue,
        targetMotorAngle,
        ignoreCheckPoint,
    )


def followLinePidSimplified(
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensor: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetLRI: int,
    targetCheckPointLRI: int,
    currentKpValue: float,
    currentKiValue: float,
    currentKdValue: float,
    targetMotorAngle: int,
    ignoreCheckPoint: Boolean,
):
    """Follows the line using the PID controller for a certain number of degrees.

    Args:
        lineFollowingColorSensor (ColorSensor): [description]
        checkpointColorSensor (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        currentDriveSpeed (int): [description]
        targetLRI (int): [description]
        targetCheckPointLRI (int): [description]
        currentKpValue (float): [description]
        currentKiValue (float): [description]
        currentKdValue (float): [description]
        targetMotorAngle (int): [description]
        ignoreCheckPoint (Boolean): [description]
    """
    # For detecting the black bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = targetCheckPointLRI
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = targetLRI
    # Parameters for PID line flollowing.
    KP_VALUE = currentKpValue
    KI_VALUE = currentKiValue
    KD_VALUE = currentKdValue
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True

    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    constants.robot.stop(Stop.BRAKE)
    constants.leftMotor.reset_angle(0)
    constants.robot.drive(currentDriveSpeed, 0)

    #
    while True:
        currentLeftMotorAngle = constants.leftMotor.angle()
        if currentLeftMotorAngle >= targetMotorAngle:
            break
        # If the checkpoint color sensor hits a black bar,
        # which means we reached the destination, then we stop the Robot

        if (not ignoreCheckPoint) and (
            checkpointColorSensor.reflection() <= TARGET_CHECKPOINT_LIGHT_REFLECTION
        ):
            print(
                "PIDLineFollower.followLinePidSimplified: target reached. current left motor angle=",
                currentLeftMotorAngle,
            )
            break

        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        # Set the drive base speed and turn rate.
        constants.robot.drive(currentDriveSpeed, PIDValue)

    constants.robot.stop(Stop.BRAKE)


def whiteCheckpoint(
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensor: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetLRI: int,
    targetCheckPointLRI: int,
    currentKpValue: float,
    currentKiValue: float,
    currentKdValue: float,
    targetMotorAngle: int,
    ignoreCheckPoint: Boolean,
):
    """Follows the line using the PID controller for a certain number of degrees.

    Args:
        lineFollowingColorSensor (ColorSensor): [description]
        checkpointColorSensor (ColorSensor): [description]
        followLeftEdge (Boolean): [description]
        currentDriveSpeed (int): [description]
        targetLRI (int): [description]
        targetCheckPointLRI (int): [description]
        currentKpValue (float): [description]
        currentKiValue (float): [description]
        currentKdValue (float): [description]
        targetMotorAngle (int): [description]
        ignoreCheckPoint (Boolean): [description]
    """
    # For detecting the white bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = targetCheckPointLRI
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = targetLRI
    # Parameters for PID line flollowing.
    KP_VALUE = currentKpValue
    KI_VALUE = currentKiValue
    KD_VALUE = currentKdValue
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True

    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    constants.robot.stop(Stop.BRAKE)
    constants.leftMotor.reset_angle(0)
    constants.robot.drive(currentDriveSpeed, 0)

    #
    while True:
        currentLeftMotorAngle = constants.leftMotor.angle()
        if currentLeftMotorAngle >= targetMotorAngle:
            break
        # If the checkpoint color sensor hits a black bar,
        # which means we reached the destination, then we stop the Robot

        if (not ignoreCheckPoint) and (
            checkpointColorSensor.reflection() >= TARGET_CHECKPOINT_LIGHT_REFLECTION
        ):
            print(
                "PIDLineFollower.followLinePidSimplified: target reached. current left motor angle=",
                currentLeftMotorAngle,
            )
            break

        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        # Set the drive base speed and turn rate.
        constants.robot.drive(currentDriveSpeed, PIDValue)

    constants.robot.stop(Stop.BRAKE)


def followLinePid(
    lineFollowingColorSensor: ColorSensor,
    checkpointColorSensor: ColorSensor,
    followLeftEdge: Boolean,
    currentDriveSpeed: int,
    targetMotorAngle: int,
    targetLRI: int,
    targetCheckPointLRI: int,
):
    """Follows the line using the PID controller.

    Args:
        lineFollowingColorSensor (ColorSensor): The color sensor that is used to follow the line.
        checkpointColorSensor (ColorSensor): The color sensor that is used to detect the checkpoint.
        followLeftEdge (Boolean): If true, the robot follows the left edge of the line.
        currentDriveSpeed (int): The current drive speed of the robot.
        targetMotorAngle (int): The target motor angle of the robot.
        targetLRI (int): The target light reflection index of the line.
        targetCheckPointLRI (int): The target light reflection index of the checkpoint.
    """
    # Start PID LiNE fOlLOwiNg with the COLOR SENSOR
    constants.robot.drive(currentDriveSpeed, 0)

    # For detecting the black bar.
    TARGET_CHECKPOINT_LIGHT_REFLECTION = targetCheckPointLRI
    # For detecting the midway between black and white of the line...
    TARGET_LIGHT_REFLECTION = targetLRI
    # Parameters for PID line flollowing.
    KP_VALUE = 1.1
    KI_VALUE = 0.001
    KD_VALUE = 10
    integral = 0
    currentError = 0
    pastError = 0
    isNotCloseToTargetMotorAngle = True

    #
    while True:
        currentLeftMotorAngle = constants.leftMotor.angle()
        # If the robot travels beyond the target motor angle and hits a black bar,
        # which means we reached the destination, then we stop the Robot
        if (
            checkpointColorSensor.reflection() < TARGET_CHECKPOINT_LIGHT_REFLECTION
        ) and (currentLeftMotorAngle > targetMotorAngle):
            print("current left motor angle is ", currentLeftMotorAngle)
            break

        # When the robot has travelled far enough near the target motor angle,
        # we slow down so the robot will brake right on target

        if currentLeftMotorAngle > (targetMotorAngle - 100) and (
            isNotCloseToTargetMotorAngle
        ):
            currentDriveSpeed = currentDriveSpeed / 2
            isNotCloseToTargetMotorAngle = False
            print("Felix the right motor's angle is ", currentLeftMotorAngle)

        currrentLightReflection = lineFollowingColorSensor.reflection()
        pastError = currentError

        if followLeftEdge:
            # follow the left edge of the line
            currentError = -(TARGET_LIGHT_REFLECTION - currrentLightReflection)
        else:
            # follow the right edge of the line
            currentError = TARGET_LIGHT_REFLECTION - currrentLightReflection
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = currentError * KP_VALUE + integral * KI_VALUE + derivative * KD_VALUE

        # Set the drive base speed and turn rate.
        constants.robot.drive(currentDriveSpeed, PIDValue)

    constants.robot.stop(Stop.BRAKE)


def driveUntilCertainGyroAngle(
    driveSpeed: int,
    driveSteering: int,
    turnAngle: float,
    isClockwise: Boolean,
):
    """Drives the robot until the gyro sensor reaches a certain angle.

    Args:
        driveSpeed (int): The current drive speed of the robot.
        driveSteering (int): The current steering of the robot.
        turnAngle (float): The target angle of the robot.
        isClockwise (Boolean): If true, the robot turns clockwise.
    """
    constants.robot.stop(Stop.BRAKE)

    gyroAngleBeforeTurn = constants.gyroSensor.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print("driveUntilCertainGyroAngle: speed=", driveSpeed, "steering=", driveSteering)
    constants.robot.drive(driveSpeed, driveSteering)
    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        if isClockwise and constants.gyroSensor.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and constants.gyroSensor.angle() <= gyroAngleAfterTurn:
            break

    constants.robot.stop(Stop.BRAKE)
    print("driveUntilCertainGyroAngle: gyro angle=", constants.gyroSensor.angle())


def turnWithGyroSensorGuidanceAndColorSensor(
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
    targetColorSensor: ColorSensor,
    targetLRI: float,
):
    """Turns the robot with the gyro sensor guidance and color sensor guidance.

    Args:
        turnAngle (int): The target angle of the robot.
        turnTime (int): The time that the robot will turn.
        turnRadius (float): The radius of the turn.
        isClockwise (Boolean): If true, the robot turns clockwise.
        targetColorSensor (ColorSensor): The color sensor that is used to detect the target.
        targetLRI (float): The target light reflection index of the target.
    """

    if isClockwise:
        steering = turnAngle * (1000 / turnTime)
    else:
        steering = -(turnAngle * (1000 / turnTime))
    speed = 2 * math.pi * (turnRadius) * (turnAngle / 360) * (1000 / turnTime)

    gyroAngleBeforeTurn = constants.gyroSensor.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print(
        "speed=",
        speed,
        "steering=",
        steering,
        "rightColorLRI=",
        targetColorSensor.reflection(),
    )
    constants.robot.drive(speed, steering)
    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        if targetColorSensor.reflection() <= targetLRI:
            break
        if isClockwise and constants.gyroSensor.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and constants.gyroSensor.angle() <= gyroAngleAfterTurn:
            break
    constants.robot.stop(Stop.BRAKE)


def driveForCertainMotorAngle(
    currentMotor: Motor,
    robotSpeed: float,
    robotSteering: float,
    targetMotorAngle: float,
):
    """Drives the robot until the motor reaches a certain angle.

    Args:
        currentMotor (Motor): The motor that is used to drive.
        robotSpeed (float): The current drive speed of the robot.
        robotSteering (float): The current steering of the robot.
        targetMotorAngle (float): The target motor angle of the robot.
    """
    currentMotor.reset_angle(0)
    constants.robot.drive(robotSpeed, robotSteering)

    while True:
        if (
            robotSpeed < 0
        ) and currentMotor.angle() <= targetMotorAngle:  # robot drives backwards and it reaches the target angle
            break
        elif (robotSpeed > 0) and (
            currentMotor.angle() >= targetMotorAngle
        ):  # robot drives forwards and it reaches the target angle
            break

    constants.robot.stop(Stop.BRAKE)

#!/usr/bin/env pybricks-micropython
from helpers.ColorSensorHelper import calibrateColorSensor
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
from helpers import constants, ColorSensorHelper
import time, math


def turnWithGyroSensorRampingDown(
    targetTurnAngle: float,
    initialTurnSpeed: float,
):
    """Turn the robot with the gyro sensor while ramping down.

    Args:
        targetTurnAngle (float): The target turn angle.
        initialTurnSpeed (float): The initial turn speed.
    """
    constants.robot.stop(Stop.BRAKE)
    startingAngle = constants.gyroSensor.angle()
    currentAngle = startingAngle
    currentTurnAngle = currentAngle - startingAngle

    output = "{currentTime}, {turnSpeed}, {angle}"

    startTime = time.time()
    while True:
        # target turn angle reached, get out of the loop
        if abs(currentTurnAngle - targetTurnAngle) < 0.5:
            break

        currentGyroAngle = constants.gyroSensor.angle()
        currentTurnAngle = currentGyroAngle - startingAngle

        # Speed ramping down according to cosine curve (from 0 degree to 90 degree, or maxSpeed (initialSpeed * cos(0)), to zero speed (initialSpeed * cos(90)))
        adjustedAngle = (abs(currentTurnAngle) / targetTurnAngle) * 90
        currentTurnSpeed = initialTurnSpeed * math.cos(math.radians(adjustedAngle))

        elapsedTime = (time.time() - startTime) * 1000
        print(
            output.format(
                currentTime=elapsedTime,
                turnSpeed=currentTurnSpeed,
                angle=currentTurnAngle,
            )
        )

        # Spin turn
        if (
            currentTurnAngle <= targetTurnAngle
        ):  # turn closewise, and the outer wheel (which is the left wheel) needs to turn faster for tighter turn. Reason Unknown.
            constants.leftMotor.run(currentTurnSpeed)
            constants.rightMotor.run(-currentTurnSpeed)
        else:  # turn counter-clockwise, and the outer wheel (which is the right wheel) needs to turn slower for tighter turn. Reason unknown.
            constants.rightMotor.run(currentTurnSpeed)
            constants.leftMotor.run(-currentTurnSpeed)

    constants.robot.stop(Stop.BRAKE)

    elapsedTime = time.time() - startTime

    print(
        "Robot Stopped. The current Degree of the turn=",
        currentTurnAngle,
        ", elapsed time=",
        elapsedTime,
        ", gyro=",
        constants.gyroSensor.angle(),
        ", starting gyro=",
        startingAngle,
    )


def pidTurnInPlaceWithGyro(
    targetTurnAngle: float,
    currentKpValue: float,
    currentKiValue: float,
    currentKdValue: float,
    isClockwise: Boolean,
):
    """Turn the robot in place with the gyro sensor with PID.

    Args:
        targetTurnAngle (float): The target turn angle.
        currentKpValue (float): The current Kp value.
        currentKiValue (float): The current Ki value.
        currentKdValue (float): The current Kd value.
        isClockwise (Boolean): The direction of the turn.
    """ """"""
    # Parameters for PID gyro turning (optimized values for 90 degree clockwise turn: Kp=10.1,Ki=0.003, Kd=1.1)
    integral = 0
    currentError = 0
    pastError = 0

    constants.robot.stop(Stop.BRAKE)
    startingAngle = constants.gyroSensor.angle()
    currentAngle = startingAngle
    currentAngleTurned = 0
    steering = 0

    output = "{currentTime}, {PIDVal}"

    startTime = time.time()
    while True:
        # target turn angle reached, get out of the loop
        if abs(abs(currentAngleTurned) - targetTurnAngle) < 0.5:
            break

        currentGyroAngle = constants.gyroSensor.angle()
        currentAngleTurned = currentGyroAngle - startingAngle
        pastError = currentError

        currentError = targetTurnAngle - abs(currentAngleTurned)
        integral = integral + currentError
        derivative = currentError - pastError
        PIDValue = (
            currentError * currentKpValue
            + integral * currentKiValue
            + derivative * currentKdValue
        )
        if isClockwise:
            steering = PIDValue / 2
        else:
            steering = -PIDValue / 2

        output = "currentError={error}, integral={inte}, derivative={deri}, PIDValue={PIDVal}, steering={steeringVal}"

        print(
            output.format(
                error=currentError,
                inte=integral,
                deri=derivative,
                PIDVal=PIDValue,
                steeringVal=steering,
            )
        )

        elapsedTime = time.time() - startTime
        # print(output.format(currentTime=elapsedTime, PIDVal=PIDValue))

        # Spin turn or turn in place
        constants.robot.drive(0, steering)

    constants.robot.stop(Stop.BRAKE)

    elapsedTime = time.time() - startTime

    print(
        "Robot Stopped. The current Degree of the turn=",
        currentAngleTurned,
        ", elapsed time=",
        elapsedTime,
        ", gyro=",
        constants.gyroSensor.angle(),
        ", starting gyro=",
        startingAngle,
    )


def simpleTurnInPlaceWithGyro(
    turnAngle: float,
    turnSpeed: float,
    isClockwise: Boolean,
):
    """Turn in place with gyro (0 radius)

    Args:
        turnAngle (float): The angle to turn
        turnSpeed (float): The speed to turn
        isClockwise (Boolean): What direction to turn
    """
    constants.robot.stop(Stop.BRAKE)

    if isClockwise:
        steering = turnSpeed
    else:
        steering = -turnSpeed

    gyroAngleBeforeTurn = constants.gyroSensor.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print("simpleTurnInPlaceWithGyro: steering=", steering)
    constants.robot.drive(0, steering)
    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        if isClockwise and currentGyroAngle >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyroAngle <= gyroAngleAfterTurn:
            break

    constants.robot.stop(Stop.BRAKE)
    print("simpleTurnInPlaceWithGyro: gyro angle=", constants.gyroSensor.angle())


def simpleTurnInPlaceRampingDown(
    turnAngle: float,
    turnSpeed: float,
    isClockwise: Boolean,
):
    """Turn in place while slowing down

    Args:
        turnAngle (float): The angle to turn
        turnSpeed (float): The speed while turning
        isClockwise (Boolean): [description]
    """
    constants.robot.stop(Stop.BRAKE)

    if isClockwise:
        steering = turnSpeed
    else:
        steering = -turnSpeed

    gyroAngleBeforeTurn = constants.gyroSensor.angle()
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + turnAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - turnAngle
    print("simpleTurnInPlaceRampingDown: steering=", steering)
    constants.robot.drive(0, steering)
    isSpeedReduced = False
    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        degreeAlreadyTurned = abs(currentGyroAngle - gyroAngleAfterTurn)
        if isClockwise and currentGyroAngle >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and currentGyroAngle <= gyroAngleAfterTurn:
            break
        # reduce the speed to half when the turning is 80% complete
        if not (isSpeedReduced) and (degreeAlreadyTurned >= (0.8 * turnAngle)):
            reducedSpeed = steering / 3
            constants.robot.drive(0, reducedSpeed)
            isSpeedReduced = True
            print(
                "simpleTurnInPlaceRampingDown: reduce turning speed to ", reducedSpeed
            )

    constants.robot.stop(Stop.BRAKE)
    print("simpleTurnInPlaceRampingDown: gyro angle=", constants.gyroSensor.angle())


def simpleTurnWithGyro(
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """Turn the robot with the gyro sensor.

    Args:
        turnAngle (int): The turn angle.
        turnTime (int): The turn time.
        turnRadius (float): The turn radius.
        isClockwise (Boolean): The direction of the turn.
    """
    constants.robot.stop(Stop.BRAKE)

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
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    constants.robot.drive(speed, steering)
    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        if isClockwise and constants.gyroSensor.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and constants.gyroSensor.angle() <= gyroAngleAfterTurn:
            break

    constants.robot.stop(Stop.BRAKE)
    print("turn_with_gyro_sensor_guidance: gyro angle=", constants.gyroSensor.angle())


def variedSpeedTurnWithGyro(
    turnAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """Varied speed turn with the gyro sensor.

    Args:
        turnAngle (int): The turn angle.
        turnTime (int): The turn time.
        turnRadius (float): The turn radius.
        isClockwise (Boolean): The direction of the turn.
    """
    constants.robot.stop(Stop.BRAKE)

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
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    constants.robot.drive(speed, steering)
    while True:
        currentGyroAngle = constants.gyroSensor.angle()

        if isClockwise and constants.gyroSensor.angle() >= gyroAngleAfterTurn:
            break
        if not (isClockwise) and constants.gyroSensor.angle() <= gyroAngleAfterTurn:
            break

    constants.robot.stop(Stop.BRAKE)
    print("turn_with_gyro_sensor_guidance: gyro angle=", constants.gyroSensor.angle())


def trueTurn(
    targetAngle: int,
    turnTime: int,
    turnRadius: float,
    isClockwise: Boolean,
):
    """True turn with gyro sensor.
    Args:
        targetAngle (int): The target angle.
        turnTime (int): The turn time.
        turnRadius (float): The turn radius.
        isClockwise (Boolean): The direction of the turn.
    """
    halfTarget = 0.5 * targetAngle
    startingGyroAngle = constants.gyroSensor.angle()
    constants.robot.stop(Stop.BRAKE)
    moveBool = True

    if isClockwise:
        steering = targetAngle * (1000 / turnTime)
    else:
        steering = -(targetAngle * (1000 / turnTime))
    speed = 2 * math.pi * (turnRadius) * (targetAngle / 360) * (1000 / turnTime)

    gyroAngleBeforeTurn = constants.gyroSensor.angle()
    # TODO: is this useless
    if isClockwise:
        gyroAngleAfterTurn = gyroAngleBeforeTurn + targetAngle
    else:
        gyroAngleAfterTurn = gyroAngleBeforeTurn - targetAngle
    print("turn_with_gyro_sensor_guidance: speed=", speed, "steering=", steering)
    constants.robot.drive(speed, steering)

    while True:
        currentGyroAngle = constants.gyroSensor.angle()
        angleTurnSoFar = currentGyroAngle - startingGyroAngle

        if angleTurnSoFar >= targetAngle:
            break

        if angleTurnSoFar >= halfTarget and moveBool:
            moveBool = False
            steering = 0.5 * steering
            print("Angle turn so far = ", angleTurnSoFar, " steering = ", steering)
            constants.robot.drive(speed, steering)

    constants.robot.stop(Stop.BRAKE)


def moveStraightUntilEitherCheckpointHitFirst(
    firstSide: str,
    firstSensor: ColorSensor,
    secondSide: str,
    secondSensor: ColorSensor,
    drivespeed: float,
):
    constants.robot.drive(drivespeed, 0)
    firstTargetLRI = ColorSensorHelper.getLRIOfColorSensor(firstSide, "white")
    secondTargetLRI = ColorSensorHelper.getLRIOfColorSensor(secondSide, "white")
    print("moveStraightUntilEitherCheckpointHitFirst: firstTargetLRI=", firstTargetLRI)
    print(
        "moveStraightUntilEitherCheckpointHitFirst: secondTargetLRI=", secondTargetLRI
    )
    while True:
        firstCS = firstSensor.reflection()
        secondCS = secondSensor.reflection()
        print("moveStraightUntilEitherCheckpointHitFirst: current firstCS=", firstCS)
        print("moveStraightUntilEitherCheckpointHitFirst: current secondCS=", secondCS)
        if (firstCS >= firstTargetLRI) or (secondCS >= secondTargetLRI):
            constants.robot.stop(Stop.BRAKE)
            break


def moveStraightUntilWhiteCheckpoint(side: str, sensor: ColorSensor, drivespeed: float):
    constants.robot.drive(drivespeed, 0)
    targetLRI = ColorSensorHelper.getLRIOfColorSensor(side, "white")
    print("moveStraightUntilWhiteCheckpoint: targetLRI=", targetLRI)
    print("moveStraightUntilWhiteCheckpoint: current CS reading=", sensor.reflection())
    while True:
        if (sensor.reflection()) >= targetLRI:
            constants.robot.stop(Stop.BRAKE)
            break


def moveStraightUntilCheckpoint(side: str, sensor: ColorSensor, drivespeed: float):
    constants.robot.drive(drivespeed, 0)
    targetLRI = ColorSensorHelper.getLRIOfColorSensor(side, "black")
    print("moveStraightUntilCheckpoint: targetLRI=", targetLRI)
    print("moveStraightUntilCheckpoint: current CS reading=", sensor.reflection())
    while True:
        if (sensor.reflection()) <= targetLRI:
            constants.robot.stop(Stop.BRAKE)
            break


def moveBackUntilCheckpoint(side: str, sensor: ColorSensor, drivespeed: float):
    constants.robot.drive(drivespeed, 0)
    targetLRI = ColorSensorHelper.getLRIOfColorSensor(side, "black")
    print("moveBackUntilCheckpoint: targetLRI=", targetLRI)
    print("moveBackUntilCheckpoint: current CS reading=", sensor.reflection())
    while True:
        if (sensor.reflection()) <= targetLRI:
            constants.robot.stop(Stop.BRAKE)
            break


def moveBackUntilWhiteCheckpoint(side: str, sensor: ColorSensor, drivespeed: float):
    constants.robot.drive(drivespeed, 0)
    targetLRI = ColorSensorHelper.getLRIOfColorSensor(side, "white")
    print("moveBackUntilWhiteCheckpoint: targetLRI=", targetLRI)
    while True:
        if (sensor.reflection()) >= targetLRI:
            print(
                "moveBackUntilWhiteCheckpoint: current CS reading=", sensor.reflection()
            )
            constants.robot.stop(Stop.BRAKE)
            break


def turnInPlaceUntilCheckpoint(side: str, sensor: ColorSensor, turnSpeed: float):
    constants.robot.drive(0, turnSpeed)
    targetLRI = ColorSensorHelper.getLRIOfColorSensor(side, "black")
    print("turnInPlaceUntilCheckpoint: targetLRI=", targetLRI)
    print("turnInPlaceUntilCheckpoint: current CS reading=", sensor.reflection())
    while True:
        if (sensor.reflection()) <= targetLRI:
            constants.robot.stop(Stop.BRAKE)
            break


def turnInPlaceUntilHalfWhiteAndHalfBlackCheckpoint(
    side: str, sensor: ColorSensor, turnSpeed: float
):
    constants.robot.drive(0, turnSpeed)
    targetLRI = (
        ColorSensorHelper.getLRIOfColorSensor(side, "white")
        + ColorSensorHelper.getLRIOfColorSensor(side, "black")
    ) / 2

    print("turnInPlaceUntilHalfWhiteAndHalfBlackCheckpoint: targetLRI=", targetLRI)
    print(
        "turnInPlaceUntilHalfWhiteAndHalfBlackCheckpoint: current CS reading=",
        sensor.reflection(),
    )
    while True:
        if (sensor.reflection()) >= targetLRI:
            constants.robot.stop(Stop.BRAKE)
            break

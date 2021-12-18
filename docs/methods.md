<a name="PIDLineFollower"></a>
# PIDLineFollower

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L2)

<a name="PIDLineFollower.followLinePidForCertainMotorRotatingDegree"></a>
#### followLinePidForCertainMotorRotatingDegree

```python
followLinePidForCertainMotorRotatingDegree(lineFollowingColorSensor: ColorSensor, followLeftEdge: Boolean, targetLineFollowingLRI: int, currentDriveSpeed: int, targetMotorAngle: int, kpValue: float, kiValue: float, kdValue: float)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L19)

Follows the line using the PID controller for a certain number of degrees.

**Arguments**:

- `lineFollowingColorSensor` _ColorSensor_ - The color sensor that is used to follow the line.
- `followLeftEdge` _Boolean_ - If true, the robot follows the left edge of the line.
- `targetLineFollowingLRI` _int_ - The target light reflection index of the line.
- `currentDriveSpeed` _int_ - The current drive speed of the robot.
- `targetMotorAngle` _int_ - The target motor angle of the robot.
- `kpValue` _float_ - The value of the proportional term of the PID controller.
- `kiValue` _float_ - The value of the integral term of the PID controller.
- `kdValue` _float_ - The value of the derivative term of the PID controller.

<a name="PIDLineFollower.followLinePidSimplified"></a>
#### followLinePidSimplified

```python
followLinePidSimplified(lineFollowingColorSensor: ColorSensor, checkpointColorSensor: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetLRI: int, targetCheckPointLRI: int, currentKpValue: float, currentKiValue: float, currentKdValue: float, targetMotorAngle: int, ignoreCheckPoint: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L59)

Follows the line using the PID controller for a certain number of degrees.

**Arguments**:

- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `checkpointColorSensor` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetLRI` _int_ - [description]
- `targetCheckPointLRI` _int_ - [description]
- `currentKpValue` _float_ - [description]
- `currentKiValue` _float_ - [description]
- `currentKdValue` _float_ - [description]
- `targetMotorAngle` _int_ - [description]
- `ignoreCheckPoint` _Boolean_ - [description]

<a name="PIDLineFollower.whiteCheckpoint"></a>
#### whiteCheckpoint

```python
whiteCheckpoint(lineFollowingColorSensor: ColorSensor, checkpointColorSensor: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetLRI: int, targetCheckPointLRI: int, currentKpValue: float, currentKiValue: float, currentKdValue: float, targetMotorAngle: int, ignoreCheckPoint: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L141)

Follows the line using the PID controller for a certain number of degrees.

**Arguments**:

- `lineFollowingColorSensor` _ColorSensor_ - [description]
- `checkpointColorSensor` _ColorSensor_ - [description]
- `followLeftEdge` _Boolean_ - [description]
- `currentDriveSpeed` _int_ - [description]
- `targetLRI` _int_ - [description]
- `targetCheckPointLRI` _int_ - [description]
- `currentKpValue` _float_ - [description]
- `currentKiValue` _float_ - [description]
- `currentKdValue` _float_ - [description]
- `targetMotorAngle` _int_ - [description]
- `ignoreCheckPoint` _Boolean_ - [description]

<a name="PIDLineFollower.followLinePid"></a>
#### followLinePid

```python
followLinePid(lineFollowingColorSensor: ColorSensor, checkpointColorSensor: ColorSensor, followLeftEdge: Boolean, currentDriveSpeed: int, targetMotorAngle: int, targetLRI: int, targetCheckPointLRI: int)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L223)

Follows the line using the PID controller.

**Arguments**:

- `lineFollowingColorSensor` _ColorSensor_ - The color sensor that is used to follow the line.
- `checkpointColorSensor` _ColorSensor_ - The color sensor that is used to detect the checkpoint.
- `followLeftEdge` _Boolean_ - If true, the robot follows the left edge of the line.
- `currentDriveSpeed` _int_ - The current drive speed of the robot.
- `targetMotorAngle` _int_ - The target motor angle of the robot.
- `targetLRI` _int_ - The target light reflection index of the line.
- `targetCheckPointLRI` _int_ - The target light reflection index of the checkpoint.

<a name="PIDLineFollower.driveUntilCertainGyroAngle"></a>
#### driveUntilCertainGyroAngle

```python
driveUntilCertainGyroAngle(driveSpeed: int, driveSteering: int, turnAngle: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L299)

Drives the robot until the gyro sensor reaches a certain angle.

**Arguments**:

- `driveSpeed` _int_ - The current drive speed of the robot.
- `driveSteering` _int_ - The current steering of the robot.
- `turnAngle` _float_ - The target angle of the robot.
- `isClockwise` _Boolean_ - If true, the robot turns clockwise.

<a name="PIDLineFollower.turnWithGyroSensorGuidanceAndColorSensor"></a>
#### turnWithGyroSensorGuidanceAndColorSensor

```python
turnWithGyroSensorGuidanceAndColorSensor(turnAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean, targetColorSensor: ColorSensor, targetLRI: float)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L333)

Turns the robot with the gyro sensor guidance and color sensor guidance.

**Arguments**:

- `turnAngle` _int_ - The target angle of the robot.
- `turnTime` _int_ - The time that the robot will turn.
- `turnRadius` _float_ - The radius of the turn.
- `isClockwise` _Boolean_ - If true, the robot turns clockwise.
- `targetColorSensor` _ColorSensor_ - The color sensor that is used to detect the target.
- `targetLRI` _float_ - The target light reflection index of the target.

<a name="PIDLineFollower.driveForCertainMotorAngle"></a>
#### driveForCertainMotorAngle

```python
driveForCertainMotorAngle(currentMotor: Motor, robotSpeed: float, robotSteering: float, targetMotorAngle: float)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/PIDLineFollower.py#L383)

Drives the robot until the motor reaches a certain angle.

**Arguments**:

- `currentMotor` _Motor_ - The motor that is used to drive.
- `robotSpeed` _float_ - The current drive speed of the robot.
- `robotSteering` _float_ - The current steering of the robot.
- `targetMotorAngle` _float_ - The target motor angle of the robot.

<a name="ColorSensorHelper"></a>
# ColorSensorHelper

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L3)

<a name="ColorSensorHelper.getLightReflection"></a>
#### getLightReflection

```python
getLightReflection(currentColorSensor: ColorSensor) -> int
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L19)

Gets the LRI reading from the COLOR SENSOR.

**Arguments**:

- `currentColorSensor` _ColorSensor_ - The COLOR SENSOR to get the LRI reading from.
  

**Returns**:

- `int` - The LRI reading from the COLOR SENSOR.

<a name="ColorSensorHelper.getAllColorCalibrationData"></a>
#### getAllColorCalibrationData

```python
getAllColorCalibrationData(calibrationDataFileName: str) -> tuple
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L31)

Gets all color calibration data and returns it as a tuple.

**Arguments**:

- `calibrationDataFileName` _str_ - The name of the CSV file to get the calibration data from.
  

**Returns**:

- `tuple` - The tuple containing the color calibration data.

<a name="ColorSensorHelper.getLRIOfColorSensor"></a>
#### getLRIOfColorSensor

```python
getLRIOfColorSensor(side: str, color: str) -> int
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L58)

Returns the reading of LRI

**Arguments**:

- `calData` _[]_ - Calibration data to be read
- `side` _str_ - `right` or `left`
- `color` _str_ - `black` or `white`

**Returns**:

- `[int]` - The stored LRI

<a name="ColorSensorHelper.calibrateColorSensor"></a>
#### calibrateColorSensor

```python
calibrateColorSensor(ev3: EV3Brick, currentColorSensor: ColorSensor, leftOrRight: str, calFileName: str)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L83)

Calibrates the selected COLOR SENSOR by getting LRI readings from both White and Black areas and stores it in the CSV Data file.

**Arguments**:

- `currentColorSensor` _ColorSensor_ - The COLOR SENSOR to calibrate.
- `leftOrRight` _str_ - `left` or `right`
- `calFileName` _str_ - The name of the CSV file to store the calibration data.

<a name="ColorSensorHelper.calibrateAllColorSensors"></a>
#### calibrateAllColorSensors

```python
calibrateAllColorSensors(fileName: str)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/ColorSensorHelper.py#L152)

Takes the LRI readings from both COLOR SENSORS, and stores them in a CSV file.

**Arguments**:

- `fileName` _str_ - The name of the CSV file to store the calibration data.

<a name="constants"></a>
# constants

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/constants.py#L3)

<a name="Navigation"></a>
# Navigation

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L2)

<a name="Navigation.turnWithGyroSensorRampingDown"></a>
#### turnWithGyroSensorRampingDown

```python
turnWithGyroSensorRampingDown(targetTurnAngle: float, initialTurnSpeed: float)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L20)

Turn the robot with the gyro sensor while ramping down.

**Arguments**:

- `targetTurnAngle` _float_ - The target turn angle.
- `initialTurnSpeed` _float_ - The initial turn speed.

<a name="Navigation.simpleTurnInPlaceWithGyro"></a>
#### simpleTurnInPlaceWithGyro

```python
simpleTurnInPlaceWithGyro(turnAngle: float, turnSpeed: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L171)

Turn in place with gyro (0 radius)

**Arguments**:

- `turnAngle` _float_ - The angle to turn
- `turnSpeed` _float_ - The speed to turn
- `isClockwise` _Boolean_ - What direction to turn

<a name="Navigation.simpleTurnInPlaceRampingDown"></a>
#### simpleTurnInPlaceRampingDown

```python
simpleTurnInPlaceRampingDown(turnAngle: float, turnSpeed: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L208)

Turn in place while slowing down

**Arguments**:

- `turnAngle` _float_ - The angle to turn
- `turnSpeed` _float_ - The speed while turning
- `isClockwise` _Boolean_ - [description]

<a name="Navigation.simpleTurnWithGyro"></a>
#### simpleTurnWithGyro

```python
simpleTurnWithGyro(turnAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L255)

Turn the robot with the gyro sensor.

**Arguments**:

- `turnAngle` _int_ - The turn angle.
- `turnTime` _int_ - The turn time.
- `turnRadius` _float_ - The turn radius.
- `isClockwise` _Boolean_ - The direction of the turn.

<a name="Navigation.variedSpeedTurnWithGyro"></a>
#### variedSpeedTurnWithGyro

```python
variedSpeedTurnWithGyro(turnAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L295)

Varied speed turn with the gyro sensor.

**Arguments**:

- `turnAngle` _int_ - The turn angle.
- `turnTime` _int_ - The turn time.
- `turnRadius` _float_ - The turn radius.
- `isClockwise` _Boolean_ - The direction of the turn.

<a name="Navigation.trueTurn"></a>
#### trueTurn

```python
trueTurn(targetAngle: int, turnTime: int, turnRadius: float, isClockwise: Boolean)
```

[`View source`](https://github.com/ExplodingBricks/CargoConnect21/blob/48b0752dbd2e79b14a99cb010d1b5ea3ba3db798/helpers/Navigation.py#L336)

True turn with gyro sensor.

**Arguments**:

- `targetAngle` _int_ - The target angle.
- `turnTime` _int_ - The turn time.
- `turnRadius` _float_ - The turn radius.
- `isClockwise` _Boolean_ - The direction of the turn.


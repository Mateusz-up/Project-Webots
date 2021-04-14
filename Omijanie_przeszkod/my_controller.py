"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
from controller import Motor
from controller import DistanceSensor

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = Motor("motor.left")
rightMotor = Motor("motor.right")

# Get frontal distance sensors.
outerLeftSensor = DistanceSensor("prox.horizontal.0")
centralLeftSensor = DistanceSensor("prox.horizontal.1")
centralSensor = DistanceSensor("prox.horizontal.2")
centralRightSensor = DistanceSensor("prox.horizontal.3")
outerRightSensor = DistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

licznik = 0
skretl = 0
skretp = 0

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue()
    centralLeftSensorValue = centralLeftSensor.getValue()
    centralSensorValue = centralSensor.getValue()
    centralRightSensorValue = centralRightSensor.getValue()
    outerRightSensorValue = outerRightSensor.getValue()

    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    # leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    # rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2 - centralSensorValue)




    if centralSensorValue != 0:
        leftMotor.setVelocity(-1)
        rightMotor.setVelocity(maxMotorVelocity)
        skretl += 1
        skretp -= 1

    elif outerLeftSensorValue != 0 or centralLeftSensorValue != 0:
        leftMotor.setVelocity(maxMotorVelocity)
        rightMotor.setVelocity(-1)

        skretp += 1
        skretl -= 1
    elif outerRightSensorValue != 0 or centralRightSensorValue != 0:
        rightMotor.setVelocity(maxMotorVelocity)
        leftMotor.setVelocity(-1)

        skretl += 1
        skretp -= 1



    elif skretp > 0 and licznik == 0:
        rightMotor.setVelocity(maxMotorVelocity)
        leftMotor.setVelocity(-1)
        skretl += 1
        skretp -= 1
        licznik = 10

    elif skretl > 0 and licznik == 0:
        leftMotor.setVelocity(maxMotorVelocity)
        rightMotor.setVelocity(-1)

        skretp += 1
        skretl -= 1
        licznik = 10

    else:
        rightMotor.setVelocity(maxMotorVelocity)
        leftMotor.setVelocity(maxMotorVelocity)

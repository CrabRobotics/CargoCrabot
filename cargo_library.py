#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import cargo_library
import diagnostics

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
back_attachment_motor = Motor(Port.C)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
robot.distance_control.limits(600, 400, 100)
robot.heading_control.limits(100, 200, 100)
timer = StopWatch()



def beep():
    ev3.speaker.beep()

def reset(angle):
    robot.reset()
    gyro.reset_angle(angle)

def gyro_drive(speed, distance, angle):
    robot.reset()
    drive_distance = robot.distance()
    kp = 5
    new_speed = speed
    while drive_distance < distance:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(new_speed, turn_rate)
        drive_distance = robot.distance()
        wait(100)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    #wait(200)

def gyro_drive_until_l(speed, angle, lines):
    kp = 5
    lines_passed = 1
    while lines_passed <= lines:
        if left_sensor.reflection() <= diagnostics.Left_Sensor_Calibration:
            while left_sensor.color() != Color.WHITE:
                deviation = angle - gyro.angle()
                turn_rate = kp * deviation
                robot.drive(speed, turn_rate)      
        while left_sensor.reflection() > diagnostics.Left_Sensor_Calibration:
            deviation = angle - gyro.angle()
            turn_rate = kp * deviation
            robot.drive(speed, turn_rate)
        lines_passed = lines_passed + 1
    robot.straight(0)
    #wait(200)

def gyro_drive_until_r(speed, angle, lines):
    kp = 5
    lines_passed = 1
    while lines_passed <= lines:
        if right_sensor.reflection() <= diagnostics.Right_Sensor_Calibration:
            while right_sensor.color() != Color.WHITE:
                deviation = angle - gyro.angle()
                turn_rate = kp * deviation
                robot.drive(speed, turn_rate)      
        while right_sensor.reflection() > diagnostics.Right_Sensor_Calibration:
            deviation = angle - gyro.angle()
            turn_rate = kp * deviation
            robot.drive(speed, turn_rate)
        lines_passed = lines_passed + 1
    robot.straight(0)
    #wait(200)

def reset_on_wall():
    timer.reset()
    while timer.time() < 500:
        robot.drive(-200, 0)
    robot.straight(0)

def bw_gyro_drive(speed, distance, angle):
    robot.reset()
    drive_distance = robot.distance()
    kp = 5
    new_speed = speed * -1
    distance = distance * -1
    while drive_distance > distance:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(new_speed, turn_rate)
        drive_distance = robot.distance()
        wait(100)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    #wait(200)

def bw_gyro_drive_until_t(speed, time, angle):
    timer.reset()
    kp = 5
    new_speed = speed * -1
    while timer.time() < time:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(new_speed, turn_rate)
        wait(100)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    #wait(200)

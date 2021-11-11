#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import cargo_library

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.

ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
timer = StopWatch()

def beep():
    ev3.speaker.beep()

def reset(angle):
    robot.reset()
    gyro.reset_angle(angle)

# def turn(angle):
#     robot.turn(angle)


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
        #print("Gyro_Drive Angle =", gyro.angle())
        #print("Speed =", new_speed)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    wait(200)
    # print("Distance Settings =", robot.distance_control.limits())
    # print("Heading Settings =", robot.heading_control.limits())

def gyro_drive_until_l(speed, angle):
    # Won't work if sensor is on black
    verify = 0
    kp = 5
    while left_sensor.color() != Color.BLACK and verify < 2:
        if left_sensor.color() == Color.BLACK:
            verify = verify + 1
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(speed, turn_rate)
        #print ("Left color sensor = ",left_sensor.color())
    robot.straight(0)
    wait(200)
    print("Angle =", gyro.angle(), "Should be", angle)

def gyro_drive_until_r(speed, angle):
    # Won't work if sensor is on black
    kp = 5
    while right_sensor.color() != Color.BLACK:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(speed, turn_rate)
        #print ("Right color sensor = ",right_sensor.color())
    robot.straight(0)
    wait(200)
    print("Angle =", gyro.angle(), "Should be", angle)

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
        #print("Gyro_Drive Angle =", gyro.angle())
        #print("Speed =", new_speed)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    wait(200)

def bw_gyro_drive_until_t(speed, time, angle):
    timer.reset()
    #robot.reset()
    kp = 5
    new_speed = speed * -1
    while timer.time() < time:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(new_speed, turn_rate)
        wait(100)
        #print("Gyro_Drive Angle =", gyro.angle())
        #print("Speed =", new_speed)
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    wait(200)
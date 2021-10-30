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
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=111)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
#robot.distance_control.limits(400, 300, 100)
#robot.heading_control.limits(300,100,100)

def beep():
    ev3.speaker.beep()

def reset(angle):
    robot.reset()
    gyro.reset_angle(angle)

def turn(angle):
    robot.turn(angle)


def gyro_drive(speed, distance, angle):
    robot.reset()
    drive_distance = robot.distance()
    kp = 5
    kd = 1
    new_speed = speed
    while drive_distance < distance:
        # if drive_distance >= distance * .75 and kd >= .4:
        #     kd = kd - .2
        #     new_speed = speed * kd
            #print("kd =", kd)
        # else:
        #     print("No Change to Speed")
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
    print("Distance Settings =", robot.distance_control.limits())
    print("Heading Settings =", robot.heading_control.limits())

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
        print ("Left color sensor = ",left_sensor.color())
    robot.straight(0)
    wait(200)

def gyro_drive_until_r(speed, angle):
    # Won't work if sensor is on black
    kp = 5
    while right_sensor.color() != Color.BLACK:
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(speed, turn_rate)
        print ("Right color sensor = ",right_sensor.color())
    robot.straight(0)
    wait(200)
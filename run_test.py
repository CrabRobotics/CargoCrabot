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
front_attachment_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)
timer = StopWatch()

def test():
    b = EV3Brick.buttons.pressed()
    cargo_library.reset(0)
    gyro.reset_angle(0)
    speed = 500
    distance = 600
    angle = 0
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
        if speed == speed:
            print('Broken')
            break
    print("Angle =", gyro.angle(), "Should be", angle)
    robot.straight(0)
    wait(200)
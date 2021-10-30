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
#left_sensor = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=111)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)

def test():
    #20 secs.
    cargo_library.reset(0)
    cargo_library.gyro_drive(200, 130, 0)
    robot.turn(45)
    cargo_library.gyro_drive_until_l(600, 45)
    robot.turn(25)
    cargo_library.gyro_drive_until_r(600, 70)
    cargo_library.gyro_drive(200, 95, 70)
    robot.turn(20)
    cargo_library.gyro_drive_until_l(600, 90)
    robot.turn(-90)
    cargo_library.gyro_drive_until_l(600, 0)
    cargo_library.gyro_drive(200, 8, 0)
    robot.turn(90)
    cargo_library.gyro_drive(200, 330, 90)
    robot.straight(-100)
    robot.turn(45)
    cargo_library.gyro_drive(200, 180, 135)
    robot.turn(90)
    robot.turn(-45)
    cargo_library.gyro_drive(200, 135, 180)
    robot.turn(-90)
    cargo_library.gyro_drive(200, 200, 90)
    cargo_library.gyro_drive_until_l(600, 98)
    robot.straight(-60)
    


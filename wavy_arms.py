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
robot.distance_control.limits(300, 200, 100)
robot.heading_control.limits(100, 200, 100)
timer = StopWatch()
#Before new axle track the track was "axle_track=111"

def wavy():
#sets robot up for run
    cargo_library.reset_on_wall()
    cargo_library.reset(0)
#pushes truck to bridge
    cargo_library.gyro_drive(200, 130, 0)
    robot.turn(45)
    cargo_library.gyro_drive_until_l(600, 45)
    robot.turn(25)
    cargo_library.gyro_drive_until_r(600, 70)
    cargo_library.gyro_drive(200, 95, 70)
#pushes bridge down
    robot.turn(20)
    cargo_library.gyro_drive_until_l(600, 90)
#loading dock crane
    robot.turn(-90)
    cargo_library.gyro_drive_until_l(600, 0)
    cargo_library.gyro_drive(200, 8, 0)
    robot.turn(90)
    cargo_library.gyro_drive(200, 330, 90)
    robot.straight(-100)
#second half of bridge
    robot.turn(45)
    cargo_library.gyro_drive_until_l(200, 135)
    # cargo_library.gyro_drive(200, 180, 135)
    robot.turn(90)
#Pulls down train track
    robot.turn(-90)
    cargo_library.gyro_drive(200, 50, 135)
    cargo_library.gyro_drive_until_r(200,135)
    robot.turn(-30)
    cargo_library.gyro_drive_until_r(200, 105)
    robot.straight(-100) 
#backing up to the wall by helicopter
    robot.turn(75)
    cargo_library.bw_gyro_drive_until_t(450, 2700, 180)
    robot.stop()
    cargo_library.reset(180)
    wait(100)
#turns in to helicopter
    cargo_library.gyro_drive(100,135,180)
    robot.turn(-125)
    cargo_library.gyro_drive(100, 130, 55)
#pushes train out
    robot.straight(-75)
    robot.turn(90)
    cargo_library.gyro_drive(200, 70, 145)
    cargo_library.gyro_drive_until_l(200, 145)
    robot.turn(35)
    cargo_library.gyro_drive_until_l(200, 180)
#go back home
    cargo_library.bw_gyro_drive(600, 180, 175)
    robot.turn(95)
    cargo_library.gyro_drive(1000, 1200, 270)

    # cargo_library.gyro_drive(200, 135, 180)
    # robot.turn(-90)
    # cargo_library.gyro_drive(200, 200, 90)
    # cargo_library.gyro_drive_until_l(600, 98)
    # robot.straight(-60)
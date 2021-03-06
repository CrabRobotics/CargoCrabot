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
back_attachment_motor = Motor(Port.C)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)
timer = StopWatch()
robot.distance_control.limits(600, 400, 100)
robot.heading_control.limits(100, 200, 100)

def carlgo():
    back_attachment_motor.run_until_stalled(-75, then=Stop.BRAKE, duty_limit=None)
    #backs up into wall
    cargo_library.reset_on_wall()
    cargo_library.reset(0)
    #push front  attachment down
    front_attachment_motor.run_until_stalled(500, then=Stop.BRAKE, duty_limit=None)
    #drive to black line by airplane
    cargo_library.gyro_drive(200, 130, 0)
    robot.turn(45)
    cargo_library.gyro_drive_until_l(600, 45, 1)
    robot.turn(25)
    #Drive to cargo connect circle
    cargo_library.gyro_drive_until_r(600, 70, 1)
    cargo_library.gyro_drive(200, 95, 70)
    robot.turn(20)
    cargo_library.gyro_drive_until_l(400, 90, 3)
    #turn into cargo connect circle
    if left_sensor.color() == Color.BLACK:
        robot.turn(100)
    else:
        cargo_library.gyro_drive_until_l(-200, 90, 1)
        robot.turn(100)
    #release 1st cargo block
    front_attachment_motor.run_angle(-300, 145, then=Stop.COAST)
    #front_attachment_motor.run_until_stalled(-500, then=Stop.BRAKE)
    #turn out of cargo connect circle
    robot.turn(-105)
    #drive to black circle by train tracks
    cargo_library.gyro_drive(200, 155, 85)
    # backup to black line by clostest to home bridge piece
    cargo_library.bw_gyro_drive(600, 50, 85)
    cargo_library.gyro_drive_until_l(-300, 100, 2)
    cargo_library.gyro_drive_until_l(300, 100, 1)
    robot.turn(90)
    #drive back to wall by accident avoidence
    cargo_library.bw_gyro_drive_until_t(300, 1500, 182)
    #rest on back wall
    cargo_library.reset(0)
    cargo_library.gyro_drive(50, 3, 0)
    #turn to face accident avoidence
    robot.turn(90)
    cargo_library.reset_on_wall()
    #bw_gyro_drive(50, 35, 90)
    #back_attachment_motor.run_until_stalled(100, then=Stop.COAST, duty_limit=None)
    #cargo_library.gyro_drive(50, 30, 90)
    back_attachment_motor.run_angle(75, 145, then=Stop.COAST)
    #drive past blue line and push yellow panel down
    #robot.straight(70)
    cargo_library.gyro_drive_until_r(50, 90, 1)#may want to speed up
    
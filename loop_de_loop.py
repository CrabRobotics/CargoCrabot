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
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=111)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
robot.distance_control.limits(400, 200, 100)
robot.heading_control.limits(100, 200, 100)

def loop():
    # ev3.speaker.beep()
    # wait(100)
    # cargo_library.beep()
    # wait(100)
    # cargo_library.beep()
    # wait(100)
    # cargo_library.beep()
    front_attachment_motor.run_angle(-500, 150, then=Stop.BRAKE)
    front_attachment_motor.run_angle(500, 150, then=Stop.BRAKE)
    cargo_library.reset_on_wall()
    cargo_library.reset(0)
    #drive to 2 black line
    cargo_library.gyro_drive_until_r(600, 0, 2)
    robot.turn(56)
    front_attachment_motor.reset_angle(0)
    #get in front of engine
    front_attachment_motor.run_angle(-500, 130, then=Stop.BRAKE)
    cargo_library.gyro_drive(200, 120, 56)
    #flip engine
    front_attachment_motor.run_angle(200, 130, then=Stop.BRAKE)
    #get to cargo plane
    robot.turn(-11)
    cargo_library.bw_gyro_drive(600, 180, 45)
    robot.turn(-90)
    #unload cargo plane
    front_attachment_motor.run_angle(-10000, 150, then=Stop.BRAKE)
    #backup for golf shot
    front_attachment_motor.run_angle(500, 150, then=Stop.BRAKE)
    cargo_library.bw_gyro_drive(600, 15, -45)
    front_attachment_motor.run_angle(-10000, 175, then=Stop.BRAKE)
    wait(100)
    cargo_library.bw_gyro_drive(600, 10, -45)
    front_attachment_motor.run_angle(500, 150, then=Stop.BRAKE)
    cargo_library.gyro_drive(600, 10, -45)
    robot.turn(60)
    front_attachment_motor.run_angle(-500, 150, then=Stop.BRAKE)
    #swing golf club
    #ev3.speaker.say("FORE")
    robot.turn(-60)
    #return to home
    robot.turn(45)
    #front_attachment_motor.run_angle(500, 150, then=Stop.BRAKE)
    cargo_library.bw_gyro_drive(600, 600, 0)
    front_attachment_motor.run_angle(500, 150, then=Stop.BRAKE)
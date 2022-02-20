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
robot.distance_control.limits(300, 200, 100)
robot.heading_control.limits(100, 200, 100)

def test1():
    
    front_attachment_motor.run_angle(-75, 45, then=Stop.BRAKE)
    front_attachment_motor.run_angle(150, 75, then=Stop.BRAKE)
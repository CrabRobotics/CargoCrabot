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
#left_sensor = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
timer = StopWatch()

def test():
    # front_attachment_motor.reset_angle(0)
    # front_attachment_motor.run_angle(-500, 142, then=Stop.BRAKE)
    # wait(2000)
    #front_attachment_motor.run_angle(100, 142, then=Stop.BRAKE)
    cargo_library.reset_on_wall()
    cargo_library.reset(0)
    cargo_library.gyro_drive_until_r(600, 0, 2)
#
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
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=122)

def beep():
    ev3.speaker.beep()

def line_follower(Speed, Distance):
    BLACK = 6
    WHITE = 94
    threshold = (BLACK + WHITE) / 2
    DRIVE_SPEED = Speed #mm per second
    PROPORTIONAL_GAIN = 0.7
    #linetime = StopWatch()

    robot.reset()
    distance = robot.distance()
    while distance < Distance:
        deviation = left_sensor.reflection() - threshold
        turn_rate = PROPORTIONAL_GAIN * deviation
        robot.drive(DRIVE_SPEED, turn_rate)
        wait(1)
        distance = robot.distance()
    robot.stop()
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

def diagnostics_test():
    cargo_library.reset(0)
    volt = ev3.battery.voltage()
    if volt < 8000:
        ev3.speaker.beep(1000, 1000)
        ev3.speaker.say("Battery")
        ev3.screen.print("Battery Problem")
    g_angle = gyro.angle()
    wait(5000)
    new_g_angle = gyro.angle()
    new_g_angle = abs(new_angle)
    if new_g_angle > 1:
        ev3.speaker.beep(1000, 1000)
        ev3.speaker.say("gyro")
        ev3.screen.print("Gyro Problem")
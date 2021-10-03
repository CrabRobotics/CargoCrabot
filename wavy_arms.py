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

def wavy():
    ev3.speaker.beep()
    wait(100)
    cargo_library.beep()
    wait(100)
    cargo_library.beep()
    cargo_library.gyro_drive(300, 100, 0)
    print(robot.angle())
    robot.turn(45)
    print(robot.angle())
    cargo_library.gyro_drive(300, 700, 0)
    print(robot.angle())
    robot.turn(31)
    print(robot.angle())
    cargo_library.gyro_drive(300, 300, 0)
    print(robot.angle())
    robot.turn(-30)
    print(robot.angle())
    cargo_library.gyro_drive(300, 280, 0)
    print(robot.angle())
    robot.turn(30)
    print(robot.angle())
    cargo_library.gyro_drive(300, 350, 0)

    print(robot.settings())
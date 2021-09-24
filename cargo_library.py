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
# #left_sensor = ColorSensor(Port.S1)
robot = DriveBase(left_motor, right_motor, wheel_diameter=86.3, axle_track=111)
gyro = GyroSensor(Port.S2)

def beep():
    ev3.speaker.beep()

# def line_follower(Speed, Distance):
#     BLACK = 6
#     WHITE = 94
#     threshold = (BLACK + WHITE) / 2
#     DRIVE_SPEED = Speed #mm per second
#     PROPORTIONAL_GAIN = 0.7
#     #linetime = StopWatch()

#     robot.reset()
#     distance = robot.distance()
#     while distance < Distance:
#         deviation = left_sensor.reflection() - threshold
#         turn_rate = PROPORTIONAL_GAIN * deviation
#         robot.drive(DRIVE_SPEED, turn_rate)
#         wait(1)
#         distance = robot.distance()
#     robot.stop()


def gyro_drive(speed, distance, angle):
    gyro.reset_angle(0)
    robot.reset()
    drive_distance = robot.distance()
    kp = 5
    ka = .1
    while drive_distance < distance:
        deviation = gyro.angle() - angle
        turn_rate = kp * deviation
        acc_speed = speed * ka
        robot.drive(acc_speed, turn_rate)
        wait(200)
        drive_distance = robot.distance()
        if ka < 1:
            ka = ka + .1
        print(gyro.angle())
    #robot.stop()
    robot.straight(0)
    wait(100)
    robot.stop()
    print(gyro.angle())
    print(robot.distance_control.limits())

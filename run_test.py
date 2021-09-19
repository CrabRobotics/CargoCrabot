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
robot = DriveBase(left_motor, right_motor, wheel_diameter=86.3, axle_track=115)
gyro = GyroSensor(Port.S2)

def test():
    def gyro_drive(speed, distance, angle):
        gyro.reset_angle(0)
        robot.reset()
        drive_distance = robot.distance()
        kp = 5
        ka = .1
        kd = .1
        while drive_distance < distance:
            deviation = gyro.angle() - angle
            turn_rate = kp * deviation
            dec_speed = 0
            acc_speed = speed * ka
            dec_start_time = distance * .75
            if distance >= dec_start_time:
                robot.drive(dec_speed, turn_rate)
            else:
                robot.drive(acc_speed, turn_rate)
            wait(200)
            drive_distance = robot.distance()
            if ka < 1:
                ka = ka + .1
            if distance >= dec_start_time:
                kd = kd + .1
                dec_speed = acc_speed - (speed * kd)
            print(gyro.angle())
        #robot.stop()
        robot.straight(0)
        wait(1000)
        print(gyro.angle())
    gyro_drive(600, 1000, 0)
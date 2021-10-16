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
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=111)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
#robot.distance_control.limits(400, 300, 100)
#robot.heading_control.limits(300,100,100)

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
    robot.reset()
    drive_distance = robot.distance()
    kp = 5
    kd = 1
    new_speed = speed
    while drive_distance < distance:
        if drive_distance >= distance * .75 and kd >= .4:
            kd = kd - .2
            new_speed = speed * kd
            #print("kd =", kd)
        # else:
        #     print("No Change to Speed")
        deviation = angle - gyro.angle()
        turn_rate = kp * deviation
        robot.drive(new_speed, turn_rate)
        drive_distance = robot.distance()
        wait(100)
        #print("Gyro_Drive Angle =", gyro.angle())
        #print("Speed =", new_speed)
    robot.straight(0)
    print("Angle =", gyro.angle(), "Should be", angle)
    # print("Distance Settings =", robot.distance_control.limits())
    # print("Heading Settings =", robot.heading_control.limits())

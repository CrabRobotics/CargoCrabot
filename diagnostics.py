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
left_sensor = ColorSensor(Port.S3)
right_sensor = ColorSensor(Port.S4)
robot = DriveBase(left_motor, right_motor, wheel_diameter=79, axle_track=116)
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
robot.distance_control.limits(300, 200, 100)
robot.heading_control.limits(100, 200, 100)
timer = StopWatch()

def diagnostics_test():
    ev3.speaker.say("Starting")
    ev3.light.on(Color.YELLOW)
    cargo_library.reset(0)
    b = EV3Brick.buttons.pressed()
    while True:
        ev3.screen.print("Please move both Color Sensors above a Black Line. Press center button when ready.")
        if Button.CENTER in b:
            LB = left_sensor.reflection()
            RB = right_sensor.reflection()
            break
    while True:
        ev3.screen.print("Please move the Left Color Sensor above a Dark Blue spot. Press center button when ready.")
        if Button.CENTER in b:
            LDB = left_sensor.reflection()
            break
    while True:
        ev3.screen.print("Please move the Right Color Sensor above a Dark Blue spot. Press center button when ready.")
        if Button.CENTER in b:
            RDB = right_sensor.reflection()
            break
    global Left_Sensor_Calibration
    Left_Sensor_Calibration = (LB+LDB)/2
    global Right_Sensor_Calibration
    Right_Sensor_Calibration = (RB+RDB)/2
    volt = ev3.battery.voltage()
    if volt < 8000:
        ev3.speaker.beep(1000, 1000)
        ev3.speaker.say("Voltage")
        ev3.screen.print("Voltage Problem")
        ev3.light.on(Color.RED)
    # current = ev3.battery.current()
    # if current < 1.80
    #     ev3.speaker.beep(1000, 1000)
    #     ev3.speaker.say("Current")
    #     ev3.screen.print("Current Problem")
    #     ev3.light.on(color.RED)
    g_angle = gyro.angle()
    wait(2500)
    new_g_angle = gyro.angle()
    new_g_angle = abs(new_g_angle)
    if new_g_angle > 1:
        ev3.speaker.beep(1000, 1000)
        ev3.speaker.say("gyro")
        ev3.screen.print("Gyro Problem")
        ev3.light.on(Color.RED)
    ev3.speaker.say("Finished")
    ev3.light.on(Color.GREEN)

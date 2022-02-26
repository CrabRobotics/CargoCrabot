#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import cargo_library
import run_test
import loop_de_loop
import wavy_arms
import diagnostics
import carlgo
import run_test_2

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

# Write your program here.
diagnostics.diagnostics_test()
battery = ev3.battery.voltage()
gyro_angle = gyro.angle()
screen_print = 0
while True:
    voltage = ev3.battery.voltage()
    current = ev3.battery.current()
    gyro_angle = gyro.angle()
    if screen_print == 0:
        ev3.screen.clear()
        ev3.screen.print("Voltage: ",voltage)
        ev3.screen.print("Current: ",current)
        ev3.screen.print("Gyro Angle: ",gyro_angle)
    screen_print += 1
    if screen_print == 50:
        screen_print = 0
    b = EV3Brick.buttons.pressed()
    if Button.CENTER in b:
        loop_de_loop.loop() 
        print("center")  
    if Button.UP in b:
        wavy_arms.wavy()
        print("up") 
    if Button.DOWN in b:
        carlgo.carlgo()
        print("down")
    if Button.RIGHT in b:
        #run_test_2.test() 
        print("right")   
    if Button.LEFT in b:
        #run_test.test1() 
        print("left") 
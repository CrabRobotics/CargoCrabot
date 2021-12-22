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

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
gyro = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)

# Write your program here.

diagnostics.diagnostics_test()
battery = ev3.battery.voltage()
gyro_angle = gyro.angle()
while True:
    voltage = ev3.battery.voltage()
    current = ev3.battery.current()
    gyro_angle = gyro.angle()
    ev3.screen.print("Voltage: ",voltage)
    ev3.screen.print("Current: ",current)
    ev3.screen.print("Gyro Angle: ",gyro_angle)
    ev3.screen.clear()
    b = EV3Brick.buttons.pressed()
    # ev3.screen.print(ev3.battery.voltage())
    # ev3.screen.print(gyro.angle())
    if Button.CENTER in b:
        run_test.test()
        print("center")  
    if Button.UP in b:
        loop_de_loop.loop()
        print("up") 
    if Button.DOWN in b:
        wavy_arms.wavy() 
        print("down")
    if Button.RIGHT in b:
        run_test.test()
        print("right")   
    if Button.LEFT in b:
        run_test.test() 
        print("left") 
    # if Button.LEFT and Button.RIGHT in b:
    #     diagnostics.diagnostics_test()
    #     print(diagnostics)
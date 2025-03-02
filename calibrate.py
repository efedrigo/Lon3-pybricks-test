from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, multitask, run_task
from pybricks.parameters import Axis
import umath
from show import show
from odometer import odometer

hub = InventorHub(front_side=-Axis.X)
watch = StopWatch()
motorLeft = Motor(Port.A,positive_direction=Direction.COUNTERCLOCKWISE)
motorRight = Motor(Port.B,positive_direction=Direction.CLOCKWISE)

myOdometer = odometer(hub,watch);

start=watch.timeus()/1000; # seconds
pos = myOdometer.getPosition()
print("Initial position:",pos)

myOdometer.startCalibration()

async def main():
    await multitask(show(hub,20),myOdometer.run(),race=True)

run_task(main())

print("finishing")
pos = myOdometer.getPosition()
print("finishing2")
acc = myOdometer.getCalibrateAcceleration();
print("finishing3")
rot = myOdometer.getCalibratedRotatedAcceleration();

print("Final position:",pos)
print("Final acceleration:",acc)
print("Final rotated acceleration:",rot)

print("finished")
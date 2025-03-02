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

async def move():
    print("move start")
    loop = True;
    n=0;

    pos = myOdometer.getPosition()
    print("Initial position:",pos)

    while loop:

        t1 = watch.timeus()/1000; # seconds
        if ((t1-start)>2):
            motorLeft.run(200)
            motorRight.run(200)

        if (motorLeft.done()!=True and motorLeft.stalled()==True):
            print("Left motor stalled")
            loop=false

        if (motorRight.done()!=True and motorRight.stalled()==True):
            print("Right motor stalled")
            loop=false

        if ((t1-start)>6):
            loop=False
    
        n+=1
        await wait(10)
    
    motorLeft.hold()
    motorRight.hold()
    print("move stop")
    pos = myOdometer.getPosition()
    print("Final position 1:",pos)

    await wait(2000)

    motorLeft.stop()
    motorRight.stop()
    pos = myOdometer.getPosition()
    print("Final position 2:",pos)

    await wait(3000)
    print("move end")


pos = myOdometer.getPosition()
print("Initial position:",pos)

#run_task(move())
#run_task(show())
async def main():
    await multitask(move(),show(hub,9),myOdometer.run(),race=True)

run_task(main())

pos = myOdometer.getPosition()
print("Final position:",pos)

print("finished")
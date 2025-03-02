from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Axis

import umath

hub = InventorHub(front_side=-Axis.X)
watch = StopWatch()
motorLeft = Motor(Port.A)
motorRight = Motor(Port.B)


print("tilt",hub.imu.tilt())
print("heading",hub.imu.heading())
a=hub.imu.acceleration()
print("3D acc",a[0])
print("3D acc",a[1])
print("3D acc",a[2])

speed = [0.0,0,0]
avgspeed = [0.0,0,0]
distance = [0,0,0]
t=watch.timeus()/1000;
start=t;
loop = True;
n=0;
g=0;

print("--------------------")
while loop:
    [pitch,roll]=hub.imu.tilt()  # degrees
    pitch=pitch
    yaw = hub.imu.heading();     # degress
    acc = hub.imu.acceleration();# mm/second^2

    alpha = yaw/180.0*umath.pi;   #rad
    beta = pitch/180.0*umath.pi;   #rad
    gamma = roll/180.0*umath.pi; #rad

    R11=umath.cos(alpha)*umath.cos(beta)
    R12=umath.cos(alpha)*umath.sin(beta)*umath.sin(gamma)-umath.sin(alpha)*umath.cos(gamma)
    R13=umath.cos(alpha)*umath.sin(beta)*umath.cos(gamma)+umath.sin(alpha)*umath.sin(gamma)
    R21=umath.sin(alpha)*umath.cos(beta)
    R22=umath.sin(alpha)*umath.sin(beta)*umath.sin(gamma)+umath.cos(alpha)*umath.cos(gamma)
    R23=umath.sin(alpha)*umath.sin(beta)*umath.cos(gamma)-umath.cos(alpha)*umath.sin(gamma)
    R31=-umath.sin(beta);
    R32=umath.cos(beta)*umath.sin(gamma);
    R33=umath.cos(beta)*umath.cos(gamma);

    P0 = acc; #mm/s^2

    P1=[R11*P0[0]+R12*P0[1]+R13*P0[2],
    R21*P0[0]+R22*P0[1]+R23*P0[2],
    R31*P0[0]+R32*P0[1]+R33*P0[2]];
    #9806.65
    P1[2]=P1[2]-9640; # mm/s^2
    g += P1[2]

    a = umath.sqrt(umath.pow(P1[0],2)+
            umath.pow(P1[1],2)+umath.pow(P1[2],2))

    t1 = watch.timeus()/1000; # seconds
    deltat=(t1-t);
    t=t1
    speed[0] += P1[0]*deltat; #mm/s
    speed[1] += P1[1]*deltat; #mm/s
    speed[2] += P1[2]*deltat #mm/s
    avgspeed[0] += speed[0]
    avgspeed[1] += speed[1]
    avgspeed[2] += speed[2]

    s=umath.sqrt(umath.pow(speed[0],2)+
        umath.pow(speed[1],2)+
        umath.pow(speed[2],2));

#    if (s<0.001):
#        speed=[0,0,0]

    distance[0] += speed[0]*deltat; #mm
    distance[1] += speed[1]*deltat; #mm
    distance[2] += speed[2]*deltat; #mm

    d=umath.sqrt(umath.pow(distance[0],2)+
        umath.pow(distance[1],2)+
        umath.pow(distance[2],2));

    if (umath.fmod(t1-start,0.2)<0.002):
        print(t1,d,s,a,yaw,pitch,roll,P1[0],P1[1],P1[2])

    if ((t1-start)>2):
        motorLeft.run(-200)
        motorRight.run(200)

    if ((t1-start)>5):
        motorLeft.hold()
        motorRight.hold()

    if ((t1-start)>10):
        loop=False
    
    n+=1

print("--------------------")

print("T1:",watch.time()," ms")
print("T2:",watch.timeus()/1000," s")
print("samples:",n)
print("avg speed:",avgspeed[0]/n,avgspeed[1]/n,avgspeed[2]/n," mm/s")
print("avg g:",g/n," mm/s^2")
print("distance:",distance," mm")

print("tilt",hub.imu.tilt())
print("heading",hub.imu.heading())

print(R11,R12,R13)
print(R21,R22,R23)
print(R31,R32,R33)
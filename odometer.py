from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath

_P1 = [-0.1199069, 0.3874327, 9669.18];

class odometer():
    position = [0,0,0];
    speed = [0,0,0];
    avgspeed = [0,0,0]
    hub = 0;
    watch = 0;
    acc_acceleration = [0,0,0];
    acc_rotated = [0,0,0];
    n = 0;

    def __init__(self,hub,watch):
        print("Odometer constructor")
        self.hub = hub;
        self.watch = watch;
    
    def __str__(self):
        return "Odometer"

    def getPosition(self):
        return self.position;

    def startCalibration(self):
        self.n = 0;
        self.acc_acceleration = [0,0,0];
        self.acc_rotated = [0,0,0];

    def getCalibrateAcceleration(self):
        if (self.n == 0):
            return [0,0,0]

        val = self.acc_acceleration;
        val[0] = val[0]/self.n
        val[1] = val[1]/self.n
        val[2] = val[2]/self.n

        return val;

    def getCalibratedRotatedAcceleration(self):
        if (self.n == 0):
            return [0,0,0]

        val = self.acc_rotated;
        val[0] = val[0]/self.n
        val[1] = val[1]/self.n
        val[2] = val[2]/self.n

        return val;

    def run(self):
        t=self.watch.timeus()/1000;
        start=t;
        g=0;
        self.n=0;
        loop=True;
        P1 = [0,0,0]

        print("---- ODOMETER ----------------")
        while loop:

            await wait(0)
            t1 = self.watch.timeus()/1000; # seconds

            [pitch,roll]=self.hub.imu.tilt()  # degrees
            yaw = self.hub.imu.heading();     # degress
            acc = self.hub.imu.acceleration();# mm/second^2
            alpha = yaw/180.0*umath.pi;   #rad
            beta = pitch/180.0*umath.pi;   #rad
            gamma = roll/180.0*umath.pi; #rad

            self.acc_acceleration[0] += acc[0]
            self.acc_acceleration[1] += acc[1]
            self.acc_acceleration[2] += acc[2]

            R11=umath.cos(alpha)*umath.cos(beta)
            R12=umath.cos(alpha)*umath.sin(beta)*umath.sin(gamma)-umath.sin(alpha)*umath.cos(gamma)
            R13=umath.cos(alpha)*umath.sin(beta)*umath.cos(gamma)+umath.sin(alpha)*umath.sin(gamma)
            R21=umath.sin(alpha)*umath.cos(beta)
            R22=umath.sin(alpha)*umath.sin(beta)*umath.sin(gamma)+umath.cos(alpha)*umath.cos(gamma)
            R23=umath.sin(alpha)*umath.sin(beta)*umath.cos(gamma)-umath.cos(alpha)*umath.sin(gamma)
            R31=-umath.sin(beta);
            R32=umath.cos(beta)*umath.sin(gamma);
            R33=umath.cos(beta)*umath.cos(gamma);

            P1=[R11*acc[0]+R12*acc[1]+R13*acc[2],
                R21*acc[0]+R22*acc[1]+R23*acc[2],
                R31*acc[0]+R32*acc[1]+R33*acc[2]];

            self.acc_rotated[0] += P1[0]
            self.acc_rotated[1] += P1[1]
            self.acc_rotated[2] += P1[2]

            #9806.65
            P1[0] -=_P1[0]; # mm/s^2
            P1[1] -=_P1[1]; # mm/s^2
            P1[2] -=_P1[2]; # mm/s^2

            a = umath.sqrt(umath.pow(P1[0],2)+
                umath.pow(P1[1],2)+umath.pow(P1[2],2))

            t1 = self.watch.timeus()/1000; # seconds
            deltat=(t1-t);
            t=t1
            self.speed[0] += P1[0]*deltat; #mm/s
            self.speed[1] += P1[1]*deltat; #mm/s
            self.speed[2] += P1[2]*deltat #mm/s
            self.avgspeed[0] += self.speed[0]
            self.avgspeed[1] += self.speed[1]
            self.avgspeed[2] += self.speed[2]

            s=umath.sqrt(umath.pow(self.speed[0],2)+
                umath.pow(self.speed[1],2)+
                umath.pow(self.speed[2],2));

            self.position[0] += self.speed[0]*deltat; #mm
            self.position[1] += self.speed[1]*deltat; #mm
            self.position[2] += self.speed[2]*deltat; #mm

            d=umath.sqrt(umath.pow(self.position[0],2)+
                umath.pow(self.position[1],2)+
                umath.pow(self.position[2],2));

            if (self.n%80 == 1):
                print(self.n,t1,d,s,a,yaw,pitch,roll,P1[0],P1[1],P1[2])

            self.n += 1;







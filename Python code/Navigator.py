# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot

import time
import math
from time import sleep
import time
import globalVal
from threading import Thread
import threading
from simple_pid import PID
import statistics

class NavigatorMain(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.lock = threading.Lock()

        self.heading = 0
        self.targetHeading = 0
        self.compassOffset = 0
        self.NORMAL_MARGE = 5

        #MotorDrivePid
        Kp = 5
        Ki = 1
        Kd = 0
        self.MotorDrivePid = PID(Kp, Ki, Kd, setpoint=0)
        self.MotorDrivePid.sample_time = 0.1

        #MotorHeadingPid
        Kp = 4
        Ki = 2
        Kd = 1.5
        self.MotorHeadingPid = PID(Kp, Ki, Kd, setpoint=0)
        self.MotorHeadingPid.sample_time = 0.1
        self.MotorHeadingPid.output_limits = (-255, 255) ##Anti Windup

        self.start()

    def run(self):

        while True:
            self.motorOverwrite()
            self.checkForTask()
            time.sleep(0.1)

    def checkForTask(self):
        if globalVal.currentWaypoint != None:
            waypoint = globalVal.currentWaypoint
            pos = globalVal.IGNS_Kalman_out
            if not waypoint.isInRange(pos[0], pos[1]):
                self.dynamicMotorControlHeading(self.checkKeepMoving, waypoint.getHeading(pos[0], pos[1]), 100)
            else:
                with self.lock:
                    globalVal.currentWaypoint = None

    def deltaHoek(self,h1, h2):
        angle1 = h1 - h2
        angle2 = ((h1 + 180) % 360) - ((h2 + 180) % 360)
        if abs(angle1) < abs(angle2):
            return angle1
        return angle2

    def IMUheading(self, compass = False):
        eul = globalVal.IMU_EulXYZ[0]
        if compass:
            eul = globalVal.IMU_EulXYZ_Corrected[0]
        return eul

    def motorSturing(self, motorA, motorB):
        a = int(motorA)
        b = int(motorB)

        if abs(a - 255) > 255:
            a = 255
        if abs(b - 255) > 255:
            b = 255

        with self.lock:
            globalVal.TargetSpeedA = a
            globalVal.TargetSpeedB = b

    def goToTargetHeading(self, target, marge):
        heading = self.IMUheading(True)

        delta = self.deltaHoek(heading, target)
        pid = self.MotorHeadingPid

        while abs(delta) > abs(marge):
            while abs(delta) > abs(marge):
                sleep(0.1)
                delta = self.deltaHoek(self.IMUheading(True), target)

                control = pid(delta)
                motorA = self.minMax(255 + control, 0, 510)
                motorB = self.minMax(255 - control, 0, 510)

                self.motorSturing(motorA, motorB)

            self.motorSturing(255, 255)
            sleep(1)
            delta = self.deltaHoek(self.IMUheading(True), target)

        self.motorSturing(255, 255)

    def angleCompass(self, mag):
        heading = 180 * math.atan2(mag[1], mag[0]) / math.pi
        if heading < 0:
            heading += 360
        return heading

    def Range360(self, number):
        while number >= 360 or number < 0:
            number = number % 360
            if number < 0:
                number = 360 - number

        return number

    def ZeroCompassOffset(self):
        compassOffset = self.Range360(self.deltaHoek(0, globalVal.IMU_EulXYZ[0]))
        print("compassOfsset:", compassOffset)

        with self.lock:
            globalVal.IMU_EulCompassOffset = compassOffset

    def calibrateCompassOffset(self):

        sleep(3)
        acceptReading = False
        reading = 0
        while not acceptReading:
            tempList = []
            for i in range(500):
                tempList.append(self.angleCompass(globalVal.IMU_Compass))
                time.sleep(0.01)
            reading = statistics.median(tempList)
            dev = statistics.stdev(tempList)
            print("dev:", dev)
            if dev < 2:
                compassOffset = self.deltaHoek(0, reading) - self.deltaHoek(globalVal.IMU_Eul, 0)
                if abs(compassOffset) > 50:
                    self.goToTargetHeading(self.Range360(globalVal.IMU_EulXYZ_Corrected[0] + compassOffset), 10)
                    sleep(3)
                else:
                    acceptReading = True
            elif dev > 80:
                self.goToTargetHeading(self.Range360(globalVal.IMU_EulXYZ_Corrected[0] + 90), 10)
                sleep(3)
            else:
                sleep(1)

        print("reading:", reading)
        compassOffset = self.Range360(self.deltaHoek(0, reading) - self.deltaHoek(globalVal.IMU_Eul, 0) + 180)
        print("compassOfsset:", compassOffset)

        with self.lock:
            globalVal.IMU_EulCompassOffset = compassOffset

    def motorOverwrite(self):
        if globalVal.MqttMode == "1":
            a = 255
            b = 255

            if globalVal.MqttMotorOverwrite == 'LEFT':
                a = 0
                b = 510
            elif globalVal.MqttMotorOverwrite == 'RIGHT':
                a = 510
                b = 0
            elif globalVal.MqttMotorOverwrite == 'FOR':
                self.dynamicMotorControlHeading(self.checkKeepMoving, self.IMUheading(True), 60)
            elif globalVal.MqttMotorOverwrite.count('for') > 0:
                s = globalVal.MqttMotorOverwrite.split('for ')[1]
                head = int(s)
                self.dynamicMotorControlHeading(self.checkKeepMoving, head, 60)
            elif globalVal.MqttMotorOverwrite.count('back') > 0:
                s = globalVal.MqttMotorOverwrite.split('back ')[1]
                head = int(s)
                self.dynamicMotorControlHeading(self.checkKeepMoving, head, -100)
            elif globalVal.MqttMotorOverwrite == 'BACK':
                self.dynamicMotorControlHeading(self.checkKeepMoving, self.IMUheading(True), -100)
            elif globalVal.MqttMotorOverwrite == 'STOP':
                a = 255
                b = 255
            elif globalVal.MqttMotorOverwrite == 'compass':
                self.calibrateCompassOffset()
            elif globalVal.MqttMotorOverwrite == 'zero':
                self.ZeroCompassOffset()
                print('zero')
            elif globalVal.MqttMotorOverwrite.count('head') > 0:
                s = globalVal.MqttMotorOverwrite.split('head ')[1]
                head = int(s)
                self.goToTargetHeading(head, 5)

            self.motorSturing(a, b)
            globalVal.MqttMotorOverwrite = ''

            return True
        return False

    def checkKeepMoving(self):
        if globalVal.MqttStop:
            with self.lock:
                print("movement canceld")
                globalVal.MqttStop = False
                globalVal.currentWaypoint = None
            return False
        elif globalVal.currentWaypoint != None:
            waypoint = globalVal.currentWaypoint
            pos = globalVal.IGNS_Kalman_out
            offset = abs(self.deltaHoek(waypoint.getHeading(pos[0], pos[1]), self.IMUheading(True)))
            if waypoint.isInRange(pos[0], pos[1]) or offset > 20:
                return False
        return True

    def dynamicMotorControlHeading(self, continueMoving, heading, speed = 60):
        self.goToTargetHeading(heading, 3)
        MIN_SPEED = 70
        MAX_SPEED = 255
        speed = (MAX_SPEED - MIN_SPEED) * (speed/100) + MIN_SPEED + 255 # convert to PWN (70-255)
        motorA = speed
        motorB = speed
        pid = self.MotorDrivePid

        while continueMoving():
            sleep(0.1)
            delta = self.deltaHoek(heading, self.IMUheading(True))

            control = pid(delta)
            if control >= 0:
                motorA = speed - abs(control)
            else:
                motorB = speed - abs(control)

            self.motorSturing(motorA, motorB)

        self.motorSturing(255,255)
        print("done")
        return

    def minMax(self, x, min, max):
        if x < min:
            x = min
        if x > max:
            x = max

        return x



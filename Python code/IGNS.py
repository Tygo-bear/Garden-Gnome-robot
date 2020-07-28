# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


import math
import time
import globalVal
from threading import Thread
import threading
import queue
import numpy as np


class IGNS(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.lock = threading.Lock()
        self.predictFrequency = 100
        self.updateFrequency = 20

        self.deltaTime = 1/self.predictFrequency
        self.kalman = None

        time.sleep(5)
        globalVal.MqttMotorOverwrite = "zero"
        time.sleep(1)
        with self.lock:
            globalVal.IGNS_active = True

        self.start()

    def run(self):
        i = 0
        while True:
            while not globalVal.IGNS_active:
                time.sleep(0.001)

            self.predictPosition()
            self.IGNSPosition(self.deltaTime)

            if i * self.deltaTime >= 1/self.updateFrequency:
                i = 0
                self.updatePosition()


            time.sleep(self.deltaTime)
            i += 1

    def IGNSPosition(self, deltaT):
        Axyz = globalVal.IGNS_IMU_LinVelocity_Aligned
        Vxyz = globalVal.IGNS_velocity
        Pxyz = globalVal.IGNS_location

        if 1 < globalVal.MotorTimeMoving < 5:
            Vxyz[0] += Axyz[0] * deltaT
            Vxyz[1] += Axyz[1] * deltaT
            Vxyz[2] += Axyz[2] * deltaT
        if globalVal.Moving:
            Pxyz[0] += Vxyz[0] * deltaT
            Pxyz[1] += Vxyz[1] * deltaT
            Pxyz[2] += Vxyz[2] * deltaT
        else:
            Vxyz = [0, 0, 0]

        with self.lock:
            globalVal.IGNS_velocity = Vxyz
            globalVal.IGNS_location = Pxyz


    def GPSVeloctyUpdate(self):
        angle = globalVal.GPS_Track_angle
        vel = globalVal.GPS_Velocity
        Vx = math.cos(angle) * vel / 3.6
        Vy = math.sin(angle) * vel / 3.6
        v = [Vx, Vy, 0]

        with self.lock:
            globalVal.IGNS_GPS_Velocity = v
        return

    def IMUVelocitys(self):

        eulXYZ = globalVal.IMU_EulXYZ_Corrected
        y = eulXYZ[0] #yaw
        p = eulXYZ[1] #pitch
        r = eulXYZ[2] #roll

        c = math.cos
        s = math.sin

        mes = np.array([
            [globalVal.IMU_Lin_accel[0]*10],
            [0],
            [0]
        ])

        trans = np.array([
            [
                [c(y)*c(p), c(p) * s(y), -s(p)],
                [c(y)*s(p)*s(r)-c(r)*s(y), c(y)*c(r)+s(y)*s(r), c(p)*s(r)],
                [c(y)*c(r)*s(p)+s(y)*s(r), c(r)*s(y)*s(p)-c(y)*s(r), c(p)*c(r)]
            ]
        ])

        out = np.dot(trans, mes).tolist()[0]
        out = [out[0][0]*-1, out[1][0]*-1, out[2][0]]

        with self.lock:
            globalVal.IGNS_IMU_LinVelocity_Aligned = out

    def predictPosition(self):
        self.IMUVelocitys()
        if self.kalman is not None:

            u = np.array(
                [
                    [globalVal.IGNS_IMU_LinVelocity_Aligned[0] * 100],
                    [globalVal.IGNS_IMU_LinVelocity_Aligned[1] * 100],
                    [globalVal.IGNS_IMU_LinVelocity_Aligned[2] * 100],
                    [0],
                    [0],
                    [0],
                    [0],
                    [0],
                    [0],
                ])

            self.kalman.Q = self.BerekenVariance(globalVal.IGNS_Q_Variance) #Default
            if globalVal.Moving:
                if 1 < globalVal.MotorTimeMoving < 5:
                    variance = globalVal.IGNS_Q_Variance
                    variance[0] = 0.001
                    variance[1] = 0.001
                    variance[2] = 0.001
                    self.kalman.Q = self.BerekenVariance(variance)
                elif 1 < globalVal.MotorTimeMoving:
                    variance = globalVal.IGNS_Q_Variance
                    variance[0] = 0.0001
                    variance[1] = 0.0001
                    variance[2] = 0.0001
                    self.kalman.Q = self.BerekenVariance(variance)
                    u = np.array(
                        [
                            [0],
                            [0],
                            [0],
                            [0],
                            [0],
                            [0],
                            [0],
                            [0],
                            [0],
                        ])
            else:
                variance = globalVal.IGNS_Q_Variance
                variance[0] = 10
                variance[1] = 10
                variance[2] = 10
                self.kalman.Q = self.BerekenVariance(variance)

                u = np.array(
                    [
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                    ])

            x = self.kalman.predict(u)

            with self.lock:
                globalVal.IGNS_Kalman_out = x.T[0]
            return x.T[0]
        else:
            self.KalmanInit()
        return

    def updatePosition(self):
        if self.kalman is not None:
            self.GPSVeloctyUpdate()

            z = np.array(
                [
                    [globalVal.GPS_Loc[0]],
                    [globalVal.GPS_Loc[1]],
                    [0],
                    [globalVal.IGNS_GPS_Velocity[0]],
                    [globalVal.IGNS_GPS_Velocity[1]],
                    [globalVal.IGNS_GPS_Velocity[2]],
                    [0],
                    [0],
                    [0],
                ])

            self.kalman.R = self.BerekenVariance(globalVal.IGNS_R_Variance) #Default
            if globalVal.Moving:
                pass
            else:
                variance = globalVal.IGNS_R_Variance
                if 900 < globalVal.MotorTimeStill < 930:
                    variance[0] = 30
                    variance[1] = 30

                z = np.array(
                    [
                        [globalVal.GPS_Loc[0]],
                        [globalVal.GPS_Loc[1]],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                    ])

                variance[3] = 0.0000001
                variance[4] = 0.0000001
                variance[5] = 0.0000001

                self.kalman.R = self.BerekenVariance(variance)

            self.kalman.update(z)

    def deltaHoek(self, h1, h2):
        angle1 = h1 - h2
        angle2 = ((h1 + 180) % 360) - ((h2 + 180) % 360)
        if abs(angle1) < abs(angle2):
            return angle1
        return angle2

    def Range360(self, number):
        while number >= 360 or number < 0:
            number = number % 360
            if number < 0:
                number = 360 - number

        return number

###====================KALMAN=====================================
    def BerekenVariance(self, variance):
        varP2 = []
        for vari in variance:
            varP2.append(pow(vari, 2))

        v = varP2 * np.eye(9)
        return v

    def KalmanInit(self):
        dt = self.deltaTime
        F = np.array(
            [
                [1, 0, 0, dt, 0, 0, 0, 0, 0],  ## Location X
                [0, 1, 0, 0, dt, 0, 0, 0, 0],  ## Location Y
                [0, 0, 1, 0, 0, dt, 0, 0, 0],  ## Location Z
                [0, 0, 0, 1, 0, 0, 0, 0, 0],  ## Velocity X
                [0, 0, 0, 0, 1, 0, 0, 0, 0],  ## Velocity Y
                [0, 0, 0, 0, 0, 1, 0, 0, 0],  ## Velocity Z
                [0, 0, 0, 0, 0, 0, 1, 0, 0],  ## Orientation 1
                [0, 0, 0, 0, 0, 0, 0, 1, 0],  ## Orientation 2
                [0, 0, 0, 0, 0, 0, 0, 0, 1],  ## Orientation 3
            ])

        bDt = pow(dt, 2) / 2
        B = np.array(
            [
                [-bDt, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, bDt, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, -bDt, 0, 0, 0, 0, 0, 0],
                [-dt, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, dt, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, -dt, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
            ])

        H = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1],
            ])

        Q = self.BerekenVariance(globalVal.IGNS_Q_Variance)

        R = self.BerekenVariance(globalVal.IGNS_R_Variance)

        P = np.array(
            [
                [1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0],
            ])

        x = np.array(
            [
                [0],
                [0],
                [0],
                [0],
                [0],
                [0],
                [globalVal.IMU_EulXYZ_Corrected[0]],
                [globalVal.IMU_EulXYZ_Corrected[1]],
                [globalVal.IMU_EulXYZ_Corrected[2]],
            ]
        )

        self.kalman = KalmanFilter(F, B, H, Q, R, P, x)
        return

class KalmanFilter(object):
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P)
                        , (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)
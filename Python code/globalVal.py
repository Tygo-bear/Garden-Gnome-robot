# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


import queue
import numpy
import math
global publicTransfer

#Mqtt service
global mqttReceiveQeue
mqttReceiveQeue = queue.Queue(maxsize=20)

global mqttSend
mqttSend = queue.Queue(maxsize=20)

global MqttMode
MqttMode = "1"
global MqttMotorOverwrite
MqttMotorOverwrite = "255255"

global MqttStop
MqttStop = False

##Sensor Service
global READ_FREQUENCY
READ_FREQUENCY = 100

#GPS
global GPS_Online
GPS_Online = False
global GPS_Time
GPS_Time = 0
global GPS_Nomal
GPS_Nomal = 0
global GPS_RAW
GPS_RAW = queue.Queue(maxsize=10)
global GPS_Satellites
GPS_Satellites = 0
global GPS_DOP
GPS_DOP = (-1.-1.-1)

global GPS_Ref
GPS_Ref = (52.119, 3.038)
global GPS_Loc
GPS_Loc = [0, 0]

global GPS_Track_angle
GPS_Track_angle = 0
global GPS_Velocity
GPS_Velocity = 0 ## Km/h

#IMU
global IMU_Eul
IMU_Eul = 0

global IMU_Compass
IMU_Compass = [0, 0, 0]

global IMU_EulXYZ
IMU_EulXYZ = (0, 0, 0)

global IMU_EulXYZ_Corrected
IMU_EulXYZ_Corrected = (0, 0, 0)

global IMU_Lin_accel
IMU_Lin_accel = (0, 0, 0)

global IMU_EulCompassOffset
IMU_EulCompassOffset = 0

#Ultrasoon
global ultraSoon1
ultraSoon1 = -1
global ultraSoon2
ultraSoon2 = -1

#Motor control
global TargetSpeedA
TargetSpeedA = 255
global TargetSpeedB
TargetSpeedB = 255

global Moving
Moving = False

global MotorTimeStill
MotorTimeStill = 1000
global MotorTimeMoving
MotorTimeMoving = 0

global MFeedbackA
MFeedbackA = -1
global MFeedbackB
MFeedbackB = -1

global BatteryRawData
BatteryRawData = -1
global BatteryPercentage
BatteryPercentage = 100
global BatteryPercentageCalibration
BatteryPercentageCalibration = [677.85, 679.23, 713.04, 731.23, 744.38, 755.45, 764.23, 770.62, 777.65, 783.02, 787.1, 791.45, 794.6, 797.54, 799.64, 801.9, 803.81, 805.13, 806.73, 808.71, 809.22, 810.25, 810.64, 811.09, 812.07, 812.72, 813.72, 814.11, 814.84, 815.58, 816.52, 817.6, 818.19, 818.65, 819.64, 820.69, 821.03, 822.15, 822.57, 823.16, 824.09, 824.58, 824.67, 824.96, 825.71, 827.75, 828.41, 829.3, 829.71, 830.06, 830.47, 830.99, 831.02, 831.79, 832.64, 833.12, 833.9, 834.3, 834.85, 835.53, 835.82, 836.5, 836.67, 836.83, 837.92, 838.51, 838.93, 839.97, 840.44, 840.77, 841.2, 841.31, 841.39, 841.44, 843.81, 845.06, 845.28, 845.61, 846.12, 846.61, 846.92, 847.09, 847.53, 847.57, 848.16, 848.54, 848.76, 849.94, 849.97, 850.65, 851.24, 851.36, 851.5, 852.01, 852.2, 852.36, 853.61, 853.75, 854.45, 854.63, 854.73, 855.09, 856.02, 856.21, 856.44, 856.83, 857.12, 858.12, 858.75, 858.76, 859.41, 859.69, 860.51, 861.03, 861.21, 861.87, 861.9, 862.17, 862.46, 863.02, 863.07, 864.03, 864.33, 864.97, 865.07, 865.14, 865.33, 865.59, 865.8, 865.93, 866.36, 866.76, 867.44, 868.22, 868.34, 868.76, 868.88, 869.19, 870.18, 870.21, 871.22, 871.7, 871.79, 871.93, 872.17, 872.3, 872.33, 872.61, 872.76, 872.77, 873.03, 873.24, 873.24, 873.68, 873.75, 873.81, 873.81, 873.86, 873.86, 873.87, 873.98, 874.39, 874.75, 875.1, 875.14, 875.38, 876.99, 877.09, 877.11, 877.38, 877.53, 877.65, 877.73, 878.5, 878.9, 879.05, 879.07, 879.17, 879.31, 879.56, 879.59, 879.77, 880.26, 880.87, 881.1, 881.46, 882.36, 882.41, 882.82, 882.86, 883.01, 883.53, 883.99, 885.12, 885.67, 886.69, 888.41, 889.48, 891.22, 894.41]

global currentWaypoint
currentWaypoint = None

#IGNS
global IGNS_active
IGNS_active = False
global IGNS_location
IGNS_location = [0, 0, 0]

global IGNS_IMU_LinVelocity_Aligned
IGNS_IMU_LinVelocity_Aligned = [0, 0, 0]
global IGNS_GPS_Velocity
IGNS_GPS_Velocity = [0, 0, 0]
global IGNS_Kalman_out
IGNS_Kalman_out = numpy.array([])
global IGNS_velocity
IGNS_velocity = [0, 0, 0]

global IGNS_Q_Variance
IGNS_Q_Variance = [0.1, 0.1, 0.1, 0, 0, 0, 0.02, 0.02, 0.02]
global IGNS_R_Variance
IGNS_R_Variance = [300000, 300000, 600000, 500, 500, 0, 0, 0, 0]


#Task System
global NavTask
NavTask = None
global NavCancelTask
NavCancelTask = False


class WayPoint:
    def __init__(self, long, lat, rad):
        self.long = long
        self.lat = lat
        self.rad = rad

    def getLocal(self, refrenceLo=GPS_Ref[0], refrenceLa=GPS_Ref[1]):
        refLo = refrenceLo
        refLa = refrenceLa

        lon1 = refLo
        lat1 = refLa
        lon2 = self.long
        lat2 = self.lat

        dx = (lon1 - lon2) * 40000 * math.cos((lat1 + lat2) * math.pi / 360) / 360 * 1000
        dy = (lat1 - lat2) * 40000 / 360 * 1000

        return dx, dy

    def distenceTo(self, posX, posY):
        targetX, targetY = self.getLocal()
        dx = posX - targetX
        dy = posY - targetY
        distence = math.sqrt(pow(dx, 2) + pow(dy, 2))
        return distence

    def isInRange(self, posX, posY):
        distence = self.distenceTo(posX, posY)
        return distence <= self.rad

    def getHeading(self, posX, posY):
        x, y = self.getLocal()
        heading = math.atan2(y - posY, x - posX)
        heading = math.degrees(heading) + 180
        return heading

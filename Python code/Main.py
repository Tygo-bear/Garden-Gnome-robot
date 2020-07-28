# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


import threading
import time
import SensorService
import Manager
import Navigator
import Mqtt
import IGNS
import globalVal


def statusReportPrint():
    print("=====STATUS=====")
    print("==MQTT==")
    print("mqtt queue:", globalVal.mqttReceiveQeue)
    print("mqtt send queue:", globalVal.mqttSend)

    print()

    print("==GPS==")
    print("gps online:", globalVal.GPS_Online)
    print("gps time:", globalVal.GPS_Time)
    print("gps normal:", globalVal.GPS_Nomal)
    print("gps raw:", globalVal.GPS_RAW)

    print()

    print("==IMU==")
    print("IMU eul:", globalVal.IMU_Eul)

    print()

    print("==ulstrasoon==")
    print(" ultrasoon 1:", globalVal.ultraSoon1)
    print(" ultrasoon 2:", globalVal.ultraSoon2)

    print()

    print("==motors==")
    print(" motor A:", globalVal.TargetSpeedA)
    print(" motor B:", globalVal.TargetSpeedB)

    print()

print("Main: starting...")
SensorService.SensorMain()
IGNS.IGNS()
Navigator.NavigatorMain()
Manager.ManagerMain()
Mqtt.MqttMain()

time.sleep(2)

silent = True

while True:
    while silent:
        time.sleep(1)

    s = input("command?: ")
    if s == "status":
        statusReportPrint()
    else:
        globalVal.publicTransfer = int(s)
        time.sleep(1)




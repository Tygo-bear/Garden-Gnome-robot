# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


import time
import globalVal
from threading import Thread
import threading
import queue

class ManagerMain(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.lock = threading.Lock()

        self.start()
    def run(self):
        time.sleep(2)
        while True:
            self.readMqttCom()
            self.updateLog()
            time.sleep(1/20)

    def updateLog(self):
        l = list(globalVal.GPS_RAW.queue)
        if len(l) == 0:
            l.append((0, 0))

        if False:
            line = f"{globalVal.TargetSpeedA};{globalVal.TargetSpeedB};"
            line += f"{globalVal.IMU_Lin_accel[0]};{globalVal.IMU_Lin_accel[1]};{globalVal.IMU_Lin_accel[2]};"
            line += f"{globalVal.IGNS_IMU_LinVelocity_Aligned[0]};{globalVal.IGNS_IMU_LinVelocity_Aligned[1]};{globalVal.IGNS_IMU_LinVelocity_Aligned[2]};"
            line += f"{globalVal.IGNS_GPS_Velocity[0]};{globalVal.IGNS_GPS_Velocity[1]};"
            #line = f"{globalVal.BatteryRawData};{globalVal.BatteryPercentage}"

            with open('Log.txt', 'a') as file:
                file.write(line + "\n")


    def readMqttCom(self):
        while not globalVal.mqttReceiveQeue.empty():
            msg = globalVal.mqttReceiveQeue.get()
            self.interpreteMqttmessage(msg[0], msg[1])

    def interpreteMqttmessage(self, topic, msg):
        with self.lock:
            if topic == "mode":
                globalVal.MqttMode = msg
            elif topic == "motor_control":
                globalVal.MqttMotorOverwrite = msg
            elif topic == "stop":
                globalVal.MqttStop = True
                print("change to True")
            elif topic == 'tempChanel':
                if msg == "reset":
                    print("===IGNS location reset===")
                    globalVal.IGNS_location = [0, 0, 0]
                    print("===IGNS location reset DONE===")
                elif msg == "compass":
                    globalVal.MqttMotorOverwrite = msg
                elif msg == "zero":
                    globalVal.MqttMotorOverwrite = msg
                elif str(msg).count("head") > 0:
                    globalVal.MqttMotorOverwrite = msg
                elif str(msg).count("for") > 0:
                    globalVal.MqttMotorOverwrite = msg
                elif str(msg).count("back") > 0:
                    globalVal.MqttMotorOverwrite = msg
                print(str(msg).count("head"))
                print(msg)
            elif topic == "waypoint":
                split = msg.replace(',', '.').split(';')
                if len(split) == 3:
                    try:
                        long = float(split[0])
                        lat = float(split[1])
                        rad = float(split[2])
                        waypoint = globalVal.WayPoint(long, lat, rad)

                        globalVal.currentWaypoint = waypoint
                    except:
                        pass





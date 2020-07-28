# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


import math
import time
import queue
import globalVal
import queue
from threading import Thread
import threading
import paho.mqtt.client as mqtt
import numpy as np
np.set_printoptions(suppress=True) #prevent numpy exponential

MQTT_SERVER = "localHost"
MQTT_PATH = "motor_control"

class MqttMain(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.lock = threading.Lock()

        self.client = 0
        self.connectedRetry = True
        self.mqtt_connected = False

        self.start()

    def run(self):
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        while self.connectedRetry:
            print("Mqtt: try connecting...")
            try:
                self.client.connect(MQTT_SERVER, 1883, 60)
                self.client.loop_start()
                self.connectedRetry = False
                print("start connection mqtt")
            except:
                print("====Mqtt error ID011====")
                time.sleep(1)

        while True:
            if self.mqtt_connected:
                self.sendSensorData()
                time.sleep(0.2)
            else:
                time.sleep(1)

    def on_connect(self, client, obj, flags, rc):
        print("Connected with result code " + str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        self.client.subscribe("motor_control")
        self.client.subscribe("mode")
        self.client.subscribe("stop")
        self.client.subscribe("tempChanel")
        self.client.subscribe("waypoint")
        self.mqtt_connected = True

    def on_message(self, client, userdata, msg):
        print(msg.topic + " " + str(msg.payload))
        with self.lock:
            if globalVal.mqttReceiveQeue.maxsize <= globalVal.mqttReceiveQeue.qsize():
                globalVal.mqttReceiveQeue.get()
            globalVal.mqttReceiveQeue.put([msg.topic, str(msg.payload, 'utf-8')])

    def publish_mqqt(self, topic, msg):
        self.client.publish(topic, msg)

    def sendSensorData(self):
        self.publish_mqqt("sensors", self.getSensorData())

    def getSensorData(self):
        string = f"GPS_ONLINE:{globalVal.GPS_Online};GPS_Time:{globalVal.GPS_Time};" \
                 f"GPS_Nomal:{globalVal.GPS_Nomal};GPS_Satellites:{globalVal.GPS_Satellites};" \
                 f"GPS_DOP:{globalVal.GPS_DOP};"
        string += f"GPS_LOC:{globalVal.GPS_Loc};GPS_Track_angle:{globalVal.GPS_Track_angle};" \
                  f"GPS_Velocity:{globalVal.GPS_Velocity};;"
        string += f"IMU_Eul_corrected:{globalVal.IMU_EulXYZ_Corrected[0]};" \
                  f"IMU_EulXYZ:{globalVal.IMU_EulXYZ};IMU_EulCorrected:{globalVal.IMU_EulXYZ_Corrected};" \
                  f"IMU_Lin_accel:{globalVal.IMU_Lin_accel}"
        string += f"IMU_Compass:{self.angleCompass(globalVal.IMU_Compass)};;" \
                  f"Ultrasoon1:{globalVal.ultraSoon1};Ultrasoon2:{globalVal.ultraSoon2};;"
        string += f"IGNS_active:{globalVal.IGNS_active};"
        if globalVal.IGNS_active:
            string += f"IGNS_Kalmam:;{np.around(globalVal.IGNS_Kalman_out, decimals=2)};"
            string += f"IGNS_Location:{np.around(globalVal.IGNS_location, decimals=2)};"
        string += f"IGNS_GPS_velocity:{globalVal.IGNS_GPS_Velocity};" \
                  f"IMU_velocitys_aligned:{globalVal.IGNS_IMU_LinVelocity_Aligned};;"
        string += f"MotorTimeStill:{globalVal.MotorTimeStill};MotorTimeMoving:{globalVal.MotorTimeMoving};"
        string += f"BatteryPercentage:{globalVal.BatteryPercentage};BatteryRaw:{globalVal.BatteryRawData};"

        return string

    def angleCompass(self, mag):
        heading = 180 * math.atan2(mag[1], mag[0]) / math.pi
        if heading < 0:
            heading += 360
        return heading
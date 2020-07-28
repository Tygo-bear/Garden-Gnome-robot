# Tygo-bear
# https://github.com/Tygo-bear
# Garden Gnome Robot


from time import sleep
import globalVal
from threading import Thread
import threading
import serial
from smbus2 import SMBus
import board
import busio
import adafruit_bno055
import math
import queue

class SensorMain(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
        self.lock = threading.Lock()

        self.gpgga_info = "$GPGGA,"
        self.ser = serial.Serial("/dev/ttyS0")  # Open port with baud rate

        self.arduinoAdr = 0x08  # Adres of the Arduino
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055(self.i2c, 0x29)

        self.start()
    def run(self):
        sleep(2)
        while True:
            self.updateGPSData()
            self.updateUltrasoon()
            self.updateIMU()
            self.updateBatteryPercentage()

            self.updateMotors()
            sleep(1/(globalVal.READ_FREQUENCY*2))
            self.updateMotors()
            sleep(1/(globalVal.READ_FREQUENCY*2))

    def updateMotors(self):
        mA = globalVal.TargetSpeedA
        mB = globalVal.TargetSpeedB
        sA = '000' + str(mA)
        sB = '000' + str(mB)

        if str(mA) == '255' and str(mB) == '255':
            with self.lock:
                globalVal.Moving = False
                globalVal.MotorTimeMoving = 0
                globalVal.MotorTimeStill += 1/ globalVal.READ_FREQUENCY * 100 if globalVal.MotorTimeStill < 1000 else 0
        else:
            with self.lock:
                globalVal.Moving = True
                globalVal.MotorTimeStill = 0
                globalVal.MotorTimeMoving += 1 / globalVal.READ_FREQUENCY * 100 if globalVal.MotorTimeMoving < 1000 \
                    else 0

        self.writeData(sA[-3:] + sB[-3:])
        return

    def updateBatteryPercentage(self):
        data = self.disassembelData(self.readData())
        percentage = self.calcBatteryPercentage(int(data[6]))

        if percentage > globalVal.BatteryPercentage or globalVal.Moving:
            percentage = globalVal.BatteryPercentage

        with self.lock:
            globalVal.BatteryRawData = data[6]
            globalVal.BatteryPercentage = percentage

    def calcBatteryPercentage(self, d):
        i = 0
        for a in globalVal.BatteryPercentageCalibration:
            if d < a:
                return i/2 if i != 0 else 0
            i += 1
        return 100

    def updateIMU(self):
        try:
            eulXYZ = list(self.sensor.euler)
            eulXYZ_old = globalVal.IMU_EulXYZ
            for i in range(0, 3):
                if abs(eulXYZ[i]) > 360:
                    eulXYZ[i] = eulXYZ_old[i]
                    print("read error IMU, eul: x(0) Y(1) Z(2) ->", i)
            eulXYZ = tuple(eulXYZ)

            eul = eulXYZ[0]
            compass = self.sensor.magnetic
            accel = self.correctAcceleration(self.sensor.linear_acceleration)

            correctedEul = self.CorrectEul(eulXYZ)

            with self.lock:
                globalVal.IMU_Eul = eul
                globalVal.IMU_Compass = compass
                globalVal.IMU_EulXYZ = eulXYZ
                globalVal.IMU_Lin_accel = accel
                globalVal.IMU_EulXYZ_Corrected = tuple(correctedEul)
        except:
            print("====I2C (IMU) Error: ID003====")

    def correctAcceleration(self, accel):
        for a in accel:
            if a > 10 or a < -10:
                print("====I2C (Read) error ID005====")
                print("velocity --> " + str(a))
                return globalVal.IMU_Lin_accel
        return accel

    def CorrectEul(self, eul):
        out = [0, eul[1], 0]
        out[0] = self.CompassOffsetOnAngle(eul[0])
        out[2] = self.CorrectEulZ(eul[2])
        return out

    def CorrectEulZ(self, angle):
        if angle < 0:
            temp = 180 - abs(angle)
            temp = temp * -1
            return temp
        else:
            temp = 180 - angle
            return temp

    def CompassOffsetOnAngle(self, angle):
        return self.Range360(angle + globalVal.IMU_EulCompassOffset)

    def Range360(self, number):
        while number >= 360 or number < 0:
            number = number % 360
            if number < 0:
                number = 360 - number

        return number

    def updateUltrasoon(self):
        data = self.disassembelData(self.readData())
        with self.lock:
            globalVal.ultraSoon1 = data[4]
            globalVal.ultraSoon2 = data[5]
        return

    # strig to byte array
    def ConvertStringsToBytes(self, src):
        converted = []
        for b in src:
            converted.append(ord(b))

        return converted

    # byte array to string
    def bytesToString(self, byte):
        ret = []
        for b in byte:
            if b is not 255:
                ret.append(b)
        return bytearray(ret).decode("utf-8")

    # send data with I2C
    def writeData(self, data):
        byteToSend = self.ConvertStringsToBytes(data)  # convert to bytes

        try:
            with SMBus(1) as bus:
                bus.write_i2c_block_data(self.arduinoAdr, 0, byteToSend)
        except:
            return False

        return True

    # read data from I2C
    def readData(self):
        try:
            with SMBus(1) as bus:
                data = bus.read_i2c_block_data(self.arduinoAdr, 0, 30)
                ret = self.bytesToString(data)
                return ret
        except Exception as e:
            print(e)
            print("====I2C (Read) error ID002====")
        return

    def disassembelData(self, data):
        if data == None:
            return None
        ret = []
        temp = ""
        start = False
        for l in data:
            if l == '/':
                ret.append(temp)
                temp = ""
            elif l == '}':
                ret.append(temp)
                return ret
            elif l == '{':
                start = True
            elif start:
                temp += l
        return

    def updateGPSData(self):
        try:
            while self.ser.in_waiting > 0:
                received_data = (str)(self.ser.readline())  # read NMEA string received
                GPGGA_data_available = received_data.find(self.gpgga_info)  # check for NMEA GPGGA string

                if received_data.find("$GPGSA") > 0:
                    GPGGA_buffer = received_data.split("$GPGSA,", 1)[1]  # store data coming after "$GPGGA," string
                    NMEA_buff = (GPGGA_buffer.split(','))  # store comma separated data in buffer
                    with self.lock:
                        globalVal.GPS_DOP = (NMEA_buff[14], NMEA_buff[15], NMEA_buff[16].split("*")[0])

                if received_data.find("$GPVTG") > 0:
                    GPGGA_buffer = received_data.split("$GPVTG,", 1)[1]  # store data coming after "$GPGGA," string
                    NMEA_buff = (GPGGA_buffer.split(','))  # store comma separated data in buffer
                    with self.lock:
                        globalVal.GPS_Track_angle = float(NMEA_buff[0])
                        globalVal.GPS_Velocity = float(NMEA_buff[6])

                if (GPGGA_data_available > 0):
                    GPGGA_buffer = received_data.split("$GPGGA,", 1)[1]  # store data coming after "$GPGGA," string
                    NMEA_buff = (GPGGA_buffer.split(','))  # store comma separated data in buffer
                    self.GPS_Info(NMEA_buff)
                    return True
            return False
        except (RuntimeError, TypeError, NameError):
            print("====GPS error ID001====")
            return False

    def convert_to_degrees(self, raw_value):
        decimal_value = raw_value / 100.00
        degrees = int(decimal_value)
        mm_mmmm = (decimal_value - int(decimal_value)) / 0.6
        position = degrees + mm_mmmm
        position = "%.4f" % (position)
        return position

    def GPS_Info(self, NMEA_buff):
        nmea_time = NMEA_buff[0]  # extract time from GPGGA string
        nmea_latitude = NMEA_buff[1]  # extract latitude from GPGGA string
        nmea_longitude = NMEA_buff[3]  # extract longitude from GPGGA string
        nmea_altitude = NMEA_buff[10]
        nmea_Satellites = NMEA_buff[6] # Number of satellites

        try:
            lat = float(nmea_latitude)  # convert string into float for calculation
            longi = float(nmea_longitude)  # convertr string into float for calculation
        except ValueError:
            with self.lock:
                globalVal.GPS_Online = False
            return

        lat_in_degrees = self.convert_to_degrees(lat)  # get latitude in degree decimal format
        long_in_degrees = self.convert_to_degrees(longi)  # get longitude in degree decimal format

        with self.lock:
            globalVal.GPS_Online = True
            globalVal.GPS_Time = nmea_time
            globalVal.GPS_Satellites = nmea_Satellites

            if globalVal.GPS_RAW.maxsize <= globalVal.GPS_RAW.qsize():
                globalVal.GPS_RAW.get()
            globalVal.GPS_RAW.put([lat_in_degrees, long_in_degrees, nmea_altitude])

        self.GPS_calculateNominal()
        self.GPSCalculateLocal()
        return

    def GPS_calculateNominal(self):
        if globalVal.GPS_RAW.qsize() != globalVal.GPS_RAW.qsize():
            return

        lat = 0
        long = 0
        alt = 0
        for t in range(0, globalVal.GPS_RAW.qsize()):
            temp = globalVal.GPS_RAW.get()
            lat += float(temp[0])
            long += float(temp[1])
            alt += float(temp[2])
            globalVal.GPS_RAW.put(temp)

        lat = lat / globalVal.GPS_RAW.qsize()
        long = long / globalVal.GPS_RAW.qsize()
        alt = alt / globalVal.GPS_RAW.qsize()

        with self.lock:
            globalVal.GPS_Nomal = [lat, long, alt]

        return

    def GPSCalculateLocal(self):
        lon1 = globalVal.GPS_Ref[0]
        lat1 = globalVal.GPS_Ref[1]
        lon2 = globalVal.GPS_Nomal[0]
        lat2 = globalVal.GPS_Nomal[1]

        dx = (lon1 - lon2) * 40000 * math.cos((lat1 + lat2) * math.pi / 360) / 360 * 1000
        dy = (lat1 - lat2) * 40000 / 360 * 1000

        with self.lock:
            globalVal.GPS_Loc[0] = dx
            globalVal.GPS_Loc[1] = dy
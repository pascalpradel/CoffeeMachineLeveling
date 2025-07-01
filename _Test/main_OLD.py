from machine import Pin, I2C, PWM
import utime
import math


#############SETTINGS##############
PIN_SDA = 0 #GPIO Pin Data for Sensor
PIN_SCL = 1 #GPIO Pin Clock for Sensor
PIN_PWM = 2 #GPIO Pin for Main Servo
PIN_PWM2 = 3 #GPIO Pin for Break Servo
PIN_BTN = 4 #GPIO Pin for Button
ANGLE_FACTOR_MAIN = 0.63
ANGLE_FACTOR_BREAK = 0.88
###################################


class MPU6050(object):
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')

    def readRawData(self, reg):
        high = self.i2c.readfrom_mem(self.addr, reg, 1)[0]
        low = self.i2c.readfrom_mem(self.addr, reg+1, 1)[0]
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def getAcceleratorData(self):
        ax = self.readRawData(0x3B) / 16384.0
        ay = self.readRawData(0x3D) / 16384.0
        az = self.readRawData(0x3F) / 16384.0
        return ax, ay, az
    
    def getLevelData(self):
        ax, ay, az = self.getAcceleratorData()
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay**2 + az**2)))
        roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
        return pitch, roll

    
class Servo(object):
    def __init__(self, pin_num, min_us=500, max_us=2500, freq=50, angleFactor=0.0):
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(freq)
        self.min_us = min_us
        self.max_us = max_us
        self.angleFactor = angleFactor
        self.period = 1000000 // freq  # Periode in µs

    def setAngle(self, angle):
        angle = angle * self.angleFactor
        angle = max(0, min(180, angle))
        pulse_width = self.min_us + (angle / 180) * (self.max_us - self.min_us)
        duty_u16 = int((pulse_width * 65535) // self.period)
        self.pwm.duty_u16(duty_u16)


class CoffeMachineLeveling(object):
    def __init__(self):
        i2c = I2C(0, scl=Pin(PIN_SCL), sda=Pin(PIN_SDA), freq=400_000)
        self.mpu = MPU6050(i2c)
        self.mainservo = Servo(pin_num=PIN_PWM, angleFactor=ANGLE_FACTOR_MAIN)
        self.breakservo = Servo(pin_num=PIN_PWM2, angleFactor=ANGLE_FACTOR_BREAK)
        self.button = Pin(4, Pin.IN, Pin.PULL_UP)
        self.led = Pin("LED", Pin.OUT)
        self.multisamplingMPULevelData()

    def multisamplingMPULevelData(self, samples=5):
        pitchList = []
        rollList = []
        for i in range(samples):
            pitch, roll = self.mpu.getLevelData()
            pitchList.append(pitch)
            rollList.append(roll)
        return sum(pitchList) / len(pitchList), sum(rollList) / len(rollList)
    
    def activateBreak(self):
        self.breakservo.setAngle(90)
        utime.sleep(1)

    def deactivateBreak(self):
        self.breakservo.setAngle(50)
        utime.sleep(1)

    def setMainServoAngle(self, inputAngle):
        try:
            with open("position.txt", "r") as f:
                lastAngle = int(f.read())
        except:
            lastAngle = 90
        print("Last Angle", lastAngle)
        lastAngleDifference = lastAngle - 90
        setAngle = 90 + (inputAngle * -1) + lastAngleDifference
        with open("position.txt", "w") as f:
            f.write(str(int(setAngle)))
        print("New Angle:", setAngle)
        self.mainservo.setAngle(setAngle)
        utime.sleep(1.5)

    def start(self):
        while True:
            if self.button.value() == 0:
                self.led.toggle()
                self.deactivateBreak()
                pitch, roll = self.multisamplingMPULevelData()
                print(pitch, roll)
                self.setMainServoAngle(roll)
                self.activateBreak()
                self.led.toggle()
            else:
                utime.sleep(0.2)


main = CoffeMachineLeveling()
main.start()
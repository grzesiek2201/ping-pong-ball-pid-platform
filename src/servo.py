from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import motor


class Servo:
    i2c = busio.I2C(SCL, SDA)
    

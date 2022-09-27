#!/usr/bin/env python

import numpy as np

from board import SDA, SCL
import busio

import time

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

from pid_controller import PID
from machine_vision import MachineVision

MIN_ANGLE = 80
MAX_ANGLE = 90

START_ANGLE_1 = 80
START_ANGLE_2 = 72
START_ANGLE_3 = 80

class Platform:

    def __init__(self, show=True):

        self.init_controllers(pid_1=(0.25, 0.01, 5, 6.0, -6.0, 6.0), pid_2=(0.25, 0.01, 5, 6.0, -6.0, 6.0), pid_3=(0.3, 0.01, 6, 6.0, -6.0, 6.0))
        
        self.init_servos(START_ANGLE_1, START_ANGLE_2, START_ANGLE_3)
        
        self.mv = MachineVision(show=show)
        
    def init_controllers(self, pid_1=(0.25, 0.01, 5, 6.0, -6.0, 6.0), pid_2=(0.25, 0.01, 5, 6.0, -6.0, 6.0), 
                  pid_3=(0.3, 0.01, 6, 6.0, -6.0, 6.0)):
                      
        controller_1 = PID(P=pid_1[0], I=pid_1[1], D=pid_1[2], max_integrator=pid_1[3], min_integrator=pid_1[4], max_pid=pid_1[5])
        controller_2 = PID(P=pid_2[0], I=pid_2[1], D=pid_2[2], max_integrator=pid_2[3], min_integrator=pid_2[4], max_pid=pid_2[5])
        controller_3 = PID(P=pid_3[0], I=pid_3[1], D=pid_3[2], max_integrator=pid_3[3], min_integrator=pid_3[4], max_pid=pid_3[5])

        self.controllers = (controller_1, controller_2, controller_3)

        for controller in self.controllers:
            controller.set_point(setpoint=0)

    def init_servos(self, start_angle_1, start_angle_2, start_angle_3):
        
        self.i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50
        
        self.servo_1 = servo.Servo(self.pca.channels[15], min_pulse=500, max_pulse=2500)
        self.servo_2 = servo.Servo(self.pca.channels[14], min_pulse=500, max_pulse=2500)
        self.servo_3 = servo.Servo(self.pca.channels[13], min_pulse=500, max_pulse=2500)
        
        self.servo_1.angle = start_angle_1
        self.servo_2.angle = start_angle_2
        self.servo_3.angle = start_angle_3
    
    def one_tick(self, servo_active):
        cx, cy, error, frame = self.mv.process_frame()
        cx, cy = cx - 150, cy - 150
        #print(f"{cx = }\t{cy = }")

        pid_command_1 = self.controllers[0].update(current_value=cx/2-cy*3**(1/2)/2)
        pid_command_2 = self.controllers[1].update(current_value=cx/2+cy*3**(1/2)/2)
        pid_command_3 = self.controllers[2].update(current_value=cx)
        
        if servo_active:
            self.servo_1.angle = (START_ANGLE_1 + pid_command_1 * 1.2)
            self.servo_2.angle = (START_ANGLE_2 + pid_command_2 * 1.2)
            self.servo_3.angle = (START_ANGLE_3 - pid_command_3)
        
        #print(f"{pid_command_1 = }\t {pid_command_2 = }\t {pid_command_3 = }")
        
        return frame, cx, cy
        
    def reset_pids(self):
        for controller in self.controllers:
            controller.set_integrator(0.0)
            controller.set_derivator(0.0)
        
    def set_pids(self, pid_1, pid_2, pid_3):
        for controller, pid in zip(self.controllers, (pid_1, pid_2, pid_3)):
            controller.set_kp(pid[0])
            controller.set_ki(pid[1])
            controller.set_kd(pid[2])
        
    def get_pids(self):
        pid_1 = (self.controllers[0].get_kp(), self.controllers[0].get_ki(), self.controllers[0].get_kd())
        pid_2 = (self.controllers[1].get_kp(), self.controllers[1].get_ki(), self.controllers[1].get_kd())
        pid_3 = (self.controllers[2].get_kp(), self.controllers[2].get_ki(), self.controllers[2].get_kd())
        
        return pid_1, pid_2, pid_3

def main():
        
    platform = Platform()
    
    while 1:
        platform.one_tick(True)
        time.sleep(0.05)
        
    platform.servo_1.angle = START_ANGLE_1
    platform.servo_2.angle = START_ANGLE_2
    platform.servo_3.angle = START_ANGLE_3
    
    while 1:
        time.sleep(100)
    print("main finished")


if __name__ == '__main__':
    main()

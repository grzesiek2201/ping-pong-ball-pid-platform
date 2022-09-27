import numpy as np

class PID:

    def __init__(self, P=1.0, I=0.0, D=1.0, max_integrator=500.0, min_integrator=-500.0, max_pid=1000.0):

        self.kp = P
        self.ki = I
        self.kd = D

        # keeping track of the previous I and D values
        self.integrator = 0.0
        self.derivator = 0.0

        # anti wind-up
        self.i_max = max_integrator
        self.i_min = min_integrator
        self.max_pid = max_pid
        #print(f"{self.max_pid = }\t {max_pid = }")

        self.setpoint = None
        self.error = None

        self.max_pid = max_pid

    def update(self, current_value):
        """ Update the PID values """
        
        #print(f"Current values: {current_value}")

        self.error = (self.setpoint - current_value) / 10

        self.integrator = self.integrator + self.error

        self.p_value = self.kp * self.error
        self.i_value = self.ki * self.integrator
        self.d_value = self.kd * (self.error - self.derivator)

        # anti wind=up
        if self.i_value > self.i_max:
            self.i_value = self.i_max
        elif self.i_value < self.i_min:
            self.i_value = self.i_min

        # remember the error value
        self.derivator = self.error
        self.integrator += self.error

        pid = self.p_value + self.i_value + self.d_value
        
        if pid > self.max_pid:
            pid = self.max_pid

        if pid < -self.max_pid:
            pid = -self.max_pid

        #print(f"{round(self.setpoint, 2)=}\t{round(current_value, 2)=}\t{round(self.error, 2)=}")
        #print(f"{round(self.p_value, 2)=}\t{round(self.i_value, 2)=}\t{round(self.d_value, 2)=}")
        #print(f"{round(pid, 2) = }")

        return pid

    def set_point(self, setpoint):
        self.setpoint = setpoint
        self.integrator = 0
        self.derivator = 0

    def set_integrator(self, integrator):
        self.integrator = integrator

    def set_derivator(self, derivator):
        self.derivator = derivator

    def set_kp(self, p):
        self.kp = p

    def set_ki(self, i):
        self.ki = i

    def set_kd(self,d):
        self.kd = d
        
    def get_kp(self):
        return self.kp
        
    def get_ki(self):
        return self.ki
        
    def get_kd(self):
        return self.kd

    def get_point(self):
        return self.set_point

    def get_error(self):
        return self.error

    def get_integrator(self):
        return self.integrator

    def get_derivator(self):
        return self.derivator

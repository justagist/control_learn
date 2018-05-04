import numpy as np

class PID:

    def __init__(self, Kp = None, Kd = None, Ki = None, prev_error = 100):

        self.Kp_ = Kp
        self.Kd_ = Kd
        self.Ki_ = Ki

        self.previous_error_ = prev_error # ----- for derivative error calculation
        self.integral_ = 0

    @property
    def gain(self):
        return [self.Kp_,self.Kd_,self.Ki_]

    def find_control_input(self, error, dt = 0.01):
    # def find_control_input(self, error, dt = 0.01):

        derivative = (error - self.previous_error_) / dt
        self.integral_ += error * dt

        F = (self.Kp_ * error) + (self.Kd_ * derivative) + (self.Ki_ * self.integral_)

        self.previous_error_ = error

        return F
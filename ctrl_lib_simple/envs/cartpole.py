import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
# import sys
import ctrl_lib_simple.controllers.lqr
import ctrl_lib_simple.controllers.pid


class Cart:
    def __init__(self,x,mass,world_size):
        self.x = x  
        self.y = int(0.6*world_size)        # 0.6 was chosen for aesthetic reasons.
        self.mass = mass
        self.color = (0,255,0)

class Pendulum:
    def __init__(self,length,theta,ball_mass):
        self.length = length
        self.theta = theta
        self.ball_mass = ball_mass      
        self.color = (0,0,255)

class CartPoleEnv:

    def __init__(self, cart_mass = 5.0, pole_length = 1.0, cart_start_pos = 0.2, pendulum_mass = 1.0, pendulum_start_angle = 1.0, world_size = 1000, g = 9.81):

        self.world_size_ = world_size
        self.g_ = g

        self.cart_ = Cart(int(cart_start_pos*self.world_size_), cart_mass, self.world_size_)

        self.pendulum_ = pendulum = Pendulum(pole_length, pendulum_start_angle, pendulum_mass)


        """
            State: y = [
                        [cart_pos]
                        [cart_vel]
                        [pend_angle]
                        [pend_vel]
                        ]

        

        ----- Dynamics of the system, y_dot = Ay + Bu
        """
        self.Amat_ = np.matrix(  [
                        [0,1,0,0],
                        [0,0,self.g_*self.pendulum_.ball_mass/self.cart_.mass,0],
                        [0,0,0,1],
                        [0,0,(self.cart_.mass+self.pendulum_.ball_mass)*self.g_/(self.pendulum_.length*self.cart_.mass),0]
                        ])
        
        self.Bmat_ = np.matrix(  [
                        [0],
                        [1/self.cart_.mass],
                        [0],
                        [1/(self.pendulum_.length*self.cart_.mass)]
                        ])

    def _check_angle(self,angle):
        '''
            Function to fix the wrap-around problem with euler angles.

        '''
        previous_error = (angle % (2 * np.pi))
        if previous_error > np.pi:
            previous_error = previous_error - (2 * np.pi)
        return previous_error

    def _render(self):
        # This function displays the pendulum and cart.
        length_for_display = self.pendulum_.length * 100

        A = np.zeros((self.world_size_,self.world_size_,3),np.uint8)

        cv2.line(A,(0,int(0.6 * self.world_size_)),(self.world_size_,int(0.6 * self.world_size_)),(255,255,255),2)
        cv2.rectangle(A,(int(self.cart_.x) + 25,self.cart_.y + 15),(int(self.cart_.x) - 25,self.cart_.y - 15),self.cart_.color,-1)    

        pendulum_x_endpoint = int(self.cart_.x - (length_for_display) * np.sin(self.pendulum_.theta))
        pendulum_y_endpoint = int(self.cart_.y - (length_for_display) * np.cos(self.pendulum_.theta))

        cv2.line(A,(int(self.cart_.x),self.cart_.y),(pendulum_x_endpoint,pendulum_y_endpoint),self.pendulum_.color,4)
        cv2.circle(A,(pendulum_x_endpoint,pendulum_y_endpoint),6,(255,255,255),-1)

        cv2.imshow('WindowName',A)
        cv2.waitKey(5)


    def control_cartpole(self, control_type = 'lqr', desired_cart_pos = 0.3, desired_pendulum_angle = 0, simulation_time = 35, plot = True, **kwargs):


        def choose_ctrl(Kp = -150, Kd = -20, Ki = -20, Q = np.matrix([[10,0,0,0],
                                                                                 [0,1,0,0],
                                                                                 [0,0,10000,0],
                                                                                 [0,0,0,100]
                                                                                    ]),
                                                                            R = np.matrix([500])):


            if control_type == 'lqr':

                controller = ctrl_lib_simple.controllers.lqr.LQR()
                
                K = controller.compute_gain(self.Amat_,self.Bmat_,Q,R)
                controller.set_gain_matrix(K)

                return controller

            elif control_type == 'pid':

                controller = ctrl_lib_simple.controllers.pid.PID(Kp, Kd, Ki, prev_error = self._check_angle(self.pendulum_.theta))

                return controller

            else:
                raise Exception("Invalid Contrller Type")

        def find_control_input(controller, curr_state, desired_state, error, dt):

            if control_type == 'lqr':
                return controller.find_control_input(curr_state, desired_state)

            elif control_type == 'pid':
                return controller.find_control_input(error, dt)

            else:
                raise Exception("Invalid Contrller Type")


        def apply_control(ctrl_cmd):
            '''
                Find x and theta for the given ctrl_cmd

            '''
            theta_double_dot = (((self.cart_.mass + self.pendulum_.ball_mass) * self.g_ * np.sin(self.pendulum_.theta)) + (F * np.cos(self.pendulum_.theta)) - (self.pendulum_.ball_mass * ((theta_dot)**2.0) * self.pendulum_.length * np.sin(self.pendulum_.theta) * np.cos(self.pendulum_.theta))) / (self.pendulum_.length * (self.cart_.mass + (self.pendulum_.ball_mass * (np.sin(self.pendulum_.theta)**2.0)))) 

            x_double_dot = ((self.pendulum_.ball_mass * self.g_ * np.sin(self.pendulum_.theta) * np.cos(self.pendulum_.theta)) - (self.pendulum_.ball_mass * self.pendulum_.length * np.sin(self.pendulum_.theta) * (theta_dot**2)) + (F)) / (self.cart_.mass + (self.pendulum_.ball_mass * (np.sin(self.pendulum_.theta)**2)))

            self.cart_.x += ((time_delta**2) * x_double_dot) + (((self.cart_.x - x_tminus2) * time_delta) / previous_time_delta)

            self.pendulum_.theta += ((time_delta**2)*theta_double_dot) + (((self.pendulum_.theta - theta_tminus2)*time_delta)/previous_time_delta)


        # ----- The desired state
        desired_state = np.matrix([
                                    [desired_cart_pos*self.world_size_],
                                    [0],
                                    [desired_pendulum_angle],
                                    [0]
                                    ])

        controller = choose_ctrl(**kwargs)

        # ----- Initializing other variables needed for the simulation, x corresponds to states of the cart, theta to those of pendulum
        theta_dot = 0
        theta_tminus1 = theta_tminus2 = self.pendulum_.theta
        x_tminus1 = x_tminus2 = self.cart_.x
        previous_time_delta = 0


        # -----
        previous_timestamp = time.time()
        end_time = previous_timestamp + simulation_time


        errors, force, theta, times, x = [],[],[],[],[]

        while time.time() <= end_time:      
            current_timestamp = time.time()
            time_delta = (current_timestamp - previous_timestamp)
            error = self._check_angle(self.pendulum_.theta - desired_state[2,0])
            if previous_time_delta != 0:    # ----- This condition is to make sure that theta_dot is not infinity in the first step

                theta_dot = (theta_tminus1 - theta_tminus2 ) / previous_time_delta              
                x_dot = (x_tminus1 - x_tminus2) / previous_time_delta

                curr_state = np.matrix(  [
                    [np.squeeze(np.asarray(self.cart_.x))],
                    [np.squeeze(np.asarray(x_dot))],
                    [np.squeeze(np.asarray(self.pendulum_.theta))],
                    [np.squeeze(np.asarray(theta_dot))]
                    ])

                

                # F = controller.find_control_input(curr_state, desired_state, K = gain)
                F = find_control_input(controller, curr_state, desired_state, error, time_delta)

                apply_control(F)
                
                # For plotting the graphs
                force.append(F)
                x.append(self.cart_.x)
                errors.append(error)        
                times.append(current_timestamp)
                theta.append(self.pendulum_.theta)
        
            # Update the variables and display stuff
            self._render()
            previous_time_delta = time_delta
            previous_timestamp = current_timestamp
            theta_tminus2 = theta_tminus1
            theta_tminus1 = self.pendulum_.theta
            x_tminus2 = x_tminus1
            x_tminus1 = self.cart_.x

        if plot:

            self.plot_graphs(times,errors,theta,force,x)


    def plot_graphs(self, times,errors,theta,force,x):

        plt.subplot(4, 1, 1)
        plt.plot(times,errors,'-b')
        plt.ylabel('Error')
        plt.xlabel('Time')

        plt.subplot(4, 1, 2)
        plt.plot(times,theta,'-b')
        plt.ylabel('Theta')
        plt.xlabel('Time')

        plt.subplot(4, 1, 3)
        plt.plot(times,force,'-b')
        plt.ylabel('Force')
        plt.xlabel('Time')

        plt.subplot(4, 1, 4)
        plt.plot(times,x,'-b')
        plt.ylabel('X')
        plt.xlabel('Time')

        plt.show()



if __name__ == '__main__':
    
    cp = CartPoleEnv()

    cp.control_cartpole(control_type = 'lqr')






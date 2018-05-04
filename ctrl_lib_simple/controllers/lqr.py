import control
import numpy as np


class LQR:

    def __init__(self):

        pass

    def solve_lqr(self, A, B, Q, R):

        '''
        Returns:

            K: 2-d array : State feedback gains

            S: 2-d array : Solution to Riccati equation

            E: 1-d array : Eigenvalues of the closed loop system

        '''
        return control.lqr(A,B,Q,R)

    def compute_gain(self, A, B, Q, R):

        K, _, _ = self.solve_lqr(A,B,Q,R)
        return np.array(K)

    def find_control_input(self, curr_state, des_state, K = None):
        ''' 
            u = - K(x)

        '''

        retval = -( K * (curr_state - des_state) ) 
        return np.squeeze(np.asarray(retval))

    def compute_gain_and_find_required_control(self, A, B, Q, R, curr_state, des_state):

        return self.find_control_input(curr_state = curr_state, des_state = des_state, K = self.compute_gain(A,B,Q,R))

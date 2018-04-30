import utils.utilfuncs as utilfuncs
from dynamics import Dynamics
import numpy as np

class ContinuousSystem(Dynamics):

    def __init__(self,**kwargs):

        Dynamics.__init__(self,**kwargs)


    def check_system_stability(self, A = None):

        if A is None:
            A = self.Amat

        self._sanity_check()

        eigvals, _ = utilfuncs.get_eigen(A, eigs_as_mat = False)

        for i in eigvals:
            if isinstance(i,complex):
                if i.real > 0:
                    return False
            else:
                if i > 0:
                    return False

        return True

    def state_at(self, t):
        '''
            Get state at time t for discrete system    
        # ----- x(t) = e^(At)*x(0)
        # ----- x(t) = T*e^(Dt)*T.inv()*x(0)

        '''
        D, T = utilfuncs.get_eigen(self.Amat, eigs_as_mat = True)

        # return np.dot(T,np.dot(np.exp(D*t),np.dot(np.linalg.inv(T),self.initial_state)))
        return np.dot(np.exp(self.Amat*t),self.initial_state)

## ======================== ##
#         TEST CODE          #
## ======================== ##
if __name__ == '__main__':
    
    a = ContinuousSystem(A = np.array([[0,1],[1,-0.01]]), x = (1,2), B = np.array([[0],[1]]))

    # print a.Amat
    # print a.Bmat
    # print a.state

    # print a.get_controllability_matrix()
    # eigvals, _ = utilfuncs.get_eigen(np.array([[0,1],[-1,0.01]]), eigs_as_mat = False)

    # print eigvals

    # print a.check_controllability()

    # print a.check_system_stability()
    print a.state_at(5)
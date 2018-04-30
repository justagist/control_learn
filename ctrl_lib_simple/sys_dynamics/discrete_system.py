from dynamics import Dynamics
import numpy as np

class DiscreteSystem(Dynamics):

    def __init__(self, **kwargs):

        # ----- x(k+1) = Ax(k) + Bu

        Dynamics.__init__(self,**kwargs)



## ======================== ##
#         TEST CODE          #
## ======================== ##
if __name__ == '__main__':
    
    a = DiscreteSystem(A = np.array([[1,0],[0,1]]),x = (1,2,1,2), B = np.array([[0],[1]]))

    print a.Amat
    print a.Bmat
    print a.state
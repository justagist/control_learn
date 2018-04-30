from state_space.states import State


class Dynamics:

    def __init__(self, A = None, x = None, B = None, u = None):

        # ----- x_dot = Ax + Bu

        self.A_ = A
        self.B_ = B
        self.x_ = State(x)
        self.u_ = u

    @property
    def Amat(self):
        return self.A_

    @Amat.setter
    def Amat(self, mat):
        self.A_ = mat

    @property
    def Bmat(self):
        return self.B_

    @Bmat.setter
    def Bmat(self, mat):
        self.B_ = mat

    @property
    def state(self):
        return self.x_()

    @state.setter
    def state(self, val):
        self.x_ = State(val)

    @property
    def u(self):
        return self.u_

    @u.setter
    def u(self, val):
        self.u_ = val



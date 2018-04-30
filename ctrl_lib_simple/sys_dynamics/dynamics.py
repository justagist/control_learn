from state_space.states import State
import numpy as np
import control

class Dynamics:

    def __init__(self, A = None, x = None, B = None, u = None):

        # ----- x_dot = Ax + Bu

        self.A_ = A
        self.B_ = B
        self.state_ = State(x)
        self.u_ = np.array(u) if u is not None else None

        self.initial_state_ = self.state

        self._sanity_check()

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
        return self.state_()

    @state.setter
    def state(self, val):
        self.state_ = State(val)

    @property
    def initial_state(self):
        return self.initial_state_

    @property
    def u(self):
        return self.u_

    @u.setter
    def u(self, val):
        self.u_ = np.array(val)

    def _sanity_check(self, A = None, state = None, B = None, u = None):

        if A is None:
            A = self.Amat
        if state is None:
            state = self.state_
        if B is None:
            B = self.Bmat
        if u is None:
            u = self.u

        if A is not None and state is not None:
            assert A.shape[1] == state.dim

        if B is not None and u is not None:
            assert B.shape[1] == u.shape[0]

    def get_controllability_matrix(self, A = None, B = None):

        if A is None:
            A = self.Amat
        if B is None:
            B = self.Bmat

        return control.ctrb(A, B)

    def check_controllability(self, A = None, B = None):

        if A is None:
            A = self.Amat
        if B is None:
            B = self.Bmat

        self._sanity_check(A = A)

        return self.state_.dim == np.linalg.matrix_rank(self.get_controllability_matrix(A,B))






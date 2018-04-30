import numpy as np

class State:

    def __init__(self, dims):

        self._state = np.vstack(dims)

    def __call__(self):
        return self._state


if __name__ == '__main__':
    
    a = States((1,2,3))

    print a
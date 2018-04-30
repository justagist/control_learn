
import numpy as np

def get_eigen(mat, eigs_as_mat = False):

    eigvals, T = np.linalg.eig(mat)

    if eigs_as_mat:
        eigvals = np.diag(eigvals)

    return eigvals, T
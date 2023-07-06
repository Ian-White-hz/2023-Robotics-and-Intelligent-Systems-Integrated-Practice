import numpy as np


def pos2trans(x, y, z, alpha, beta, gamma, is_deg=False):
    if is_deg:
        alpha = alpha * np.pi / 180
        beta = beta * np.pi / 180
        gamma = gamma * np.pi / 180

    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    cg = np.cos(gamma)
    sg = np.sin(gamma)

    return np.array([
        [cb*cg, -cb*sg, sb, x],
        [sa*sb*cg+ca*sg, -sa*sb*sg+ca*cg, -sa*cb, y],
        [-ca*sb*cg+sa*sg, ca*sb*sg+sa*cg, ca*cb, z],
        [0, 0, 0, 1]
    ])


def norm_rad(rad):
    """
    Normalize the radian to [0, , 2*pi]
    """
    return np.arctan2(np.sin(rad), np.cos(rad))


def change_base(T):
    """
    Change the base of the transformation matrix.
    """
    T_new = np.zeros((4, 4))
    T_new[0:3, 0:3] = np.linalg.inv(T[0:3, 0:3])
    T_new[0:3, 3] = -T[0:3, 0:3].T.dot(T[0:3, 3])
    T_new[3, 3] = 1
    return T_new

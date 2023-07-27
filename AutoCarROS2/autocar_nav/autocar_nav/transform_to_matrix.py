import numpy as np


def transform_to_matrix(t):
    p = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    q0 = t.transform.rotation.w
    q1 = t.transform.rotation.x
    q2 = t.transform.rotation.y
    q3 = t.transform.rotation.z

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02, p[0]],
                           [r10, r11, r12, p[1]],
                           [r20, r21, r22, p[2]],
                           [0, 0, 0, 1]])

    return rot_matrix

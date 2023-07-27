import numpy as np
from geometry_msgs.msg import Quaternion

def yaw_to_quaternion(heading):
    '''
    Converts yaw heading to quaternion coordinates.
    '''
    theta = 0.5 * heading
    quaternion = Quaternion()
    quaternion.z = np.sin(theta)
    quaternion.w = np.cos(theta)

    return quaternion

def euler_from_quaternion(x, y, z, w):
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = np.arctan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = np.arcsin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = np.arctan2(t3, t4)

	return yaw_z

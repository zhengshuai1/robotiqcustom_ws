import numpy as np
from utils.transform import Rotation, Transform
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# evenly sampled time at 200ms intervals
def test1():
    v = np.arange(0.03, 0.25, 0.001)
    d = 1.6153
    # d = 0.6848
    dv = 0.001
    d_dis = dv * d / (v + dv)
    dv1 = 0.002
    d_dis1 = dv1 * d / (v + dv1)
    dv2 = 0.003
    d_dis2 = dv2 * d / (v + dv2)
    # v, d_dis2, 'b'
    plt.plot(v, d_dis, 'r')
    plt.xlabel('vel m/s')
    plt.ylabel('d error (m)')
    plt.title('d error caused by 1 mm velocity error at 1.6153 m')
    # plt.axis([0.030, 0.25, 0, 0.06])
    plt.xlim((0.030, 0.25))
    # plt.text(0.175, 0.04, r'$\Delta d= \frac{D\Delta v}{v+\Delta v}$')
    plt.grid(True)
    plt.show()

def test2(vacuum_level, d, n=1):
    force = vacuum_level * np.pi * d**2 * n / 4000
    print('pick force is', force)

test2(81.1, 17.2)

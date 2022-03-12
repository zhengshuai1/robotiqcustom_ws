from collections import defaultdict
from numpy import linalg as LA
import numpy as np
d2 = defaultdict(list)
a = [1,2, 3]
b = [4, 5]
print(a+b)

import pickle
dataList = [[1, 1, 'yes'],
            [1, 1, 'yes'],
            [1, 0, 'no'],
            [0, 1, 'no'],
            [0, 1, 'no']]
dataDic = { 0: [1, 2, 3, 4],
            1: ('a', 'b'),
            2: {'c':'yes','d':'no'}}

p = pickle.dumps(dataList)
print( p, pickle.loads(p) )
p = pickle.dumps(dataDic)
print( pickle.loads(p) )


def test_d435_fov(z):
    h_fov = np.deg2rad(69)
    v_fov = np.deg2rad(42)
    # plan 1
    region_w = 2 * z * np.tan(h_fov/2)
    region_h = 2 * z * np.tan(v_fov/2)
    plan = [region_h, region_w]
    return plan
plan = test_d435_fov(0.8)
print('dis is', plan)
a = [1, 2, 3, 4]
for key in a:
    d2[key] = []
print(d2)
def test():
    print('metal')

a = test()
print(a)

a = np.array([None, 1, None])
print('a', a)
indx = np.where(a == None)
print('indx', indx)
a = np.array([[1, np.nan, 2],
             [3, np.nan, 4]])
index = ~np.isnan(a)
b = a[index]
print(index, b)

# print(d2)
# d2["a"] = 1
# d3 = defaultdict(list)
# d3['a'] = 2
# d3['b'] = 2
# d2 = d3
# print(d2)

# def dis_test(p0, p1):
#     return LA.norm(p0 - p1)
# p0 = np.array([-0.000585943402257,-0.109966486692,0.483000010252])
# p1 = np.array([0.00208416976966, 0.108375713229, 0.485000014305])
# p1[1] = p1[1]- vel *
# val = dis_test(p0, p1)
# print('dis is',val)


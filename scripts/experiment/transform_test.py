from scipy.spatial.transform import Rotation as R

r = R.from_matrix([[0, 0, -1],
                   [0, 1, 0],
                   [1, 0, 0]])

r = R.from_matrix([[0, -1, 0],
                   [-1, 0, 0],
                   [0, 0, -1]])
print(r.as_quat())
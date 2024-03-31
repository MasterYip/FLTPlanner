import numpy as np
import pinocchio as pin

def point_SE3Act(bMa: pin.SE3, pt: np.ndarray):
    return bMa.translation + bMa.rotation @ pt

bMa = pin.SE3(pin.rpy.rpyToMatrix(np.array([1, 1, 0])), np.array([0, 0, 0]))
pt = np.array([1, 2, 3])
print(point_SE3Act(bMa, pt))

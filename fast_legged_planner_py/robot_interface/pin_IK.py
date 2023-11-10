'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-10 14:55:39
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 17:44:36
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/pin_IK.py
Description: file content
'''
# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import norm, solve

import pinocchio as pin


IK_default_settings = {
    'eps': 1e-4,        # Precision
    'IT_MAX': 1000,     # Max iterations
    'DT': 1e-1,         # Time step
    'damp': 1e-12,      # Damping
    'verbose': False
}


def pinIK(model, data, oMdes: pin.SE3, q0, id, id_mode='frame', mode=6,
             IK_settings: dict = IK_default_settings):
    """Inverse kinematics for a 6D target placement
    TODO: Solution contiuity insurement(near singularity)
    
    Args:
        model (pin.Model): model
        data (pin.Data): data
        oMdes (pin.SE3): desired placement
        q0 (np.ndarray): initial configuration
        id (int): frame id or joint id
        id_mode (str, optional): 'frame' or 'joint'. Defaults to 'frame'.
        mode (int, optional): 6D or 3D. Defaults to 6.
        IK_settings (dict, optional): Defaults to IK_default_settings.

    Raises:
        ValueError: id_mode must be frame or joint
        ValueError: mode must be 6 or 3

    Returns:
        np.ndarray: configuration q
        np.ndarray: error
        bool: success
    """
    i = 0
    q = q0.copy()
    eps = IK_settings['eps']
    IT_MAX = IK_settings['IT_MAX']
    DT = IK_settings['DT']
    damp = IK_settings['damp']
    verbose = IK_settings['verbose']
    if id_mode != 'frame' and id_mode != 'joint':
        raise ValueError('id_mode must be frame or joint')
    if mode != 6 and mode != 3:
        raise ValueError('mode must be 6 or 3')
    while True:
        pin.forwardKinematics(model, data, q)
        if id_mode == 'frame':
            iMd = pin.updateFramePlacement(model, data, id).actInv(oMdes)
        else:
            iMd = data.oMi[id].actInv(oMdes)
        if mode == 6:
            err = pin.log(iMd).vector  # in (joint) frame
        else:
            err = pin.log(iMd).vector[:3]  # FIXME: is velocity?
        
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break

        if id_mode == 'frame':
            J = pin.computeFrameJacobian(model, data, q, id)
        else:
            J = pin.computeJointJacobian(model, data, q, id)  # in joint frame
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        if mode == 6:
            v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        else:
            J = J[:3, :]
            v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
        q = pin.integrate(model, q, v*DT)
        if not i % 10 and verbose:
            print('%d: error = %s' % (i, err.T))
        i += 1
    if verbose:
        if success:
            print("Convergence achieved!")
        else:
            print(
                "\nWarning: the iterative algorithm has not reached convergence to the desired precision")
    return q, err, success

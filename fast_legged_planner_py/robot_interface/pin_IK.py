'''
Author: RaymonYip-NUC11 2205929492@qq.com
Date: 2023-11-10 14:55:39
LastEditors: RaymonYip-NUC11
LastEditTime: 2023-11-10 15:57:26
FilePath: /flplanner_ws/src/fast_legged_planner/fast_legged_planner_py/robot_interface/pin_IK.py
Description: file content
'''
# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import norm, solve

import pinocchio as pin


IK_default_settings = {
    'eps': 1e-4,
    'IT_MAX': 1000,
    'DT': 1e-1,
    'damp': 1e-12,
    'verbose': False
}


def pinIK_6D(model, data, oMdes: pin.SE3, q0, id, mode='frame',
             IK_settings: dict = IK_default_settings):
    """Inverse kinematics for a 6D target placement

    Args:
        model (pin.Model): model
        data (pin.Data): data
        oMdes (pin.SE3): desired placement
        q0 (np.ndarray): initial configuration
        id (int): frame id or joint id
        mode (str, optional): 'frame' or 'joint'. Defaults to 'frame'.
        IK_settings (dict, optional): Defaults to IK_default_settings.

    Raises:
        ValueError: mode must be frame or joint

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
    if mode != 'frame' and mode != 'joint':
        raise ValueError('mode must be frame or joint')
    while True:
        pin.forwardKinematics(model, data, q)
        if mode == 'frame':
            iMd = pin.updateFramePlacement(model, data, id).actInv(oMdes)
        else:
            iMd = data.oMi[id].actInv(oMdes)
        err = pin.log(iMd).vector  # in (joint) frame
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        if mode == 'frame':
            J = pin.computeFrameJacobian(model, data, q, id)
        else:
            # in joint frame
            J = pin.computeJointJacobian(model, data, q, id)
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
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


# FIXME: not implemented
def pinIK_3D(model, data, trans_des, q0, id, mode='frame',
             IK_settings: dict = IK_default_settings):
    """Inverse kinematics for a 3D target placement

    Args:
        model (pin.Model): model
        data (pin.Data): data
        trans_des (np.ndarray): desired position
        q0 (np.ndarray): initial configuration
        id (int): frame id or joint id
        mode (str, optional): 'frame' or 'joint'. Defaults to 'frame'.
        IK_settings (dict, optional): Defaults to IK_default_settings.

    Raises:
        ValueError: mode must be frame or joint

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
    if mode != 'frame' and mode != 'joint':
        raise ValueError('mode must be frame or joint')
    while True:
        pin.forwardKinematics(model, data, q)
        if mode == 'frame':
            err = trans_des - \
                pin.updateFramePlacement(model, data, id).translation
        else:
            err = trans_des - data.oMi[id].translation
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        if mode == 'frame':
            J = pin.computeFrameJacobian(model, data, q, id)
        else:
            # in joint frame
            J = pin.computeJointJacobian(model, data, q, id)
        J = -np.dot(pin.Jlog6(iMd.inverse()), J)
        # v:3 x 1
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
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

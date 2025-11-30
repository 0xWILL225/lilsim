"""Utility functions for working with lilsim."""

import numpy as np
from . import messages_pb2


def state_to_dict(state: messages_pb2.StateUpdate, metadata: messages_pb2.ModelMetadata) -> dict:
    """Convert a StateUpdate into named dicts using metadata."""
    data = {
        'tick': state.scene.header.tick,
        'sim_time': state.scene.header.sim_time,
        'states': {},
        'inputs': {},
        'params': {},
        'settings': {},
    }
    if metadata is None:
        return data
    for idx, value in enumerate(state.scene.state_values):
        name = metadata.states[idx].name if idx < len(metadata.states) else f'state_{idx}'
        data['states'][name] = value
    for idx, value in enumerate(state.scene.input_values):
        name = metadata.inputs[idx].name if idx < len(metadata.inputs) else f'input_{idx}'
        data['inputs'][name] = value
    for idx, value in enumerate(state.scene.param_values):
        name = metadata.params[idx].name if idx < len(metadata.params) else f'param_{idx}'
        data['params'][name] = value
    for idx, value in enumerate(state.scene.setting_values):
        name = metadata.settings[idx].name if idx < len(metadata.settings) else f'setting_{idx}'
        data['settings'][name] = int(value)
    return data

def pure_pursuit_controller(target_x: float, target_y: float, 
                            car_x: float, car_y: float, car_yaw: float,
                            lookahead: float = 2.0) -> float:
    """Simple pure pursuit steering controller.
    
    Args:
        target_x, target_y: Target point to pursue
        car_x, car_y, car_yaw: Current car state
        lookahead: Lookahead distance
        
    Returns:
        Steering angle in radians
    """
    # Transform target to car frame
    dx = target_x - car_x
    dy = target_y - car_y
    
    # Rotate to car frame
    cos_yaw = np.cos(-car_yaw)
    sin_yaw = np.sin(-car_yaw)
    local_x = dx * cos_yaw - dy * sin_yaw
    local_y = dx * sin_yaw + dy * cos_yaw
    
    # Pure pursuit formula
    curvature = 2 * local_y / (lookahead ** 2)
    
    # Assuming wheelbase of 1.0m (should match sim params)
    wheelbase = 1.0
    steer_angle = np.arctan(curvature * wheelbase)
    
    return steer_angle


def proportional_speed_controller(target_v: float, current_v: float, 
                                  kp: float = 2.0, 
                                  max_accel: float = 5.0) -> float:
    """Simple proportional speed controller.
    
    Args:
        target_v: Target velocity in m/s
        current_v: Current velocity in m/s
        kp: Proportional gain
        max_accel: Maximum acceleration magnitude
        
    Returns:
        Acceleration in m/s^2
    """
    error = target_v - current_v
    ax = kp * error
    return np.clip(ax, -max_accel, max_accel)


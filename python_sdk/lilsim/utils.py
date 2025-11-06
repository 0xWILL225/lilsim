"""Utility functions for working with lilsim."""

import numpy as np
from . import messages_pb2


def state_to_dict(state: messages_pb2.StateUpdate) -> dict:
    """Convert a StateUpdate to a dictionary for easy access.
    
    Args:
        state: StateUpdate message
        
    Returns:
        Dictionary with flattened state info
    """
    return {
        'tick': state.scene.header.tick,
        'sim_time': state.scene.header.sim_time,
        'x': state.scene.car.pos.x,
        'y': state.scene.car.pos.y,
        'yaw': state.scene.car.yaw,
        'v': state.scene.car.v,
        'yaw_rate': state.scene.car.yaw_rate,
    }


def create_line_strip(ns: str, id: int, points: np.ndarray, 
                      color: tuple = (255, 0, 0, 255),
                      scale: float = 0.05,
                      ttl_sec: float = 0.0) -> messages_pb2.Marker:
    """Create a LINE_STRIP marker from numpy array of points.
    
    Args:
        ns: Namespace
        id: Marker ID
        points: Nx2 or Nx3 array of (x, y) or (x, y, yaw) points
        color: RGBA tuple (0-255)
        scale: Line width
        ttl_sec: Time-to-live in seconds (0 = infinite)
        
    Returns:
        Marker message
    """
    marker = messages_pb2.Marker()
    marker.ns = ns
    marker.id = id
    marker.type = messages_pb2.LINE_STRIP
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.scale.x = scale
    marker.scale.y = scale
    marker.ttl_sec = ttl_sec
    marker.visible = True
    
    for point in points:
        pt = marker.points.add()
        pt.x = point[0]
        pt.y = point[1]
        pt.yaw = point[2] if len(point) > 2 else 0.0
        
    return marker


def create_circle(ns: str, id: int, x: float, y: float,
                 radius: float = 0.5,
                 color: tuple = (0, 255, 0, 255),
                 ttl_sec: float = 0.0) -> messages_pb2.Marker:
    """Create a CIRCLE marker.
    
    Args:
        ns: Namespace
        id: Marker ID
        x, y: Position
        radius: Circle radius
        color: RGBA tuple (0-255)
        ttl_sec: Time-to-live in seconds (0 = infinite)
        
    Returns:
        Marker message
    """
    marker = messages_pb2.Marker()
    marker.ns = ns
    marker.id = id
    marker.type = messages_pb2.CIRCLE
    marker.pose.x = x
    marker.pose.y = y
    marker.pose.yaw = 0.0
    marker.scale.x = radius * 2
    marker.scale.y = radius * 2
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.ttl_sec = ttl_sec
    marker.visible = True
    
    return marker


def create_text(ns: str, id: int, x: float, y: float, text: str,
               color: tuple = (255, 255, 255, 255),
               scale: float = 1.0,
               ttl_sec: float = 0.0) -> messages_pb2.Marker:
    """Create a TEXT marker.
    
    Args:
        ns: Namespace
        id: Marker ID
        x, y: Position
        text: Text content
        color: RGBA tuple (0-255)
        scale: Text scale
        ttl_sec: Time-to-live in seconds (0 = infinite)
        
    Returns:
        Marker message
    """
    marker = messages_pb2.Marker()
    marker.ns = ns
    marker.id = id
    marker.type = messages_pb2.TEXT
    marker.pose.x = x
    marker.pose.y = y
    marker.text = text
    marker.scale.x = scale
    marker.scale.y = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.ttl_sec = ttl_sec
    marker.visible = True
    
    return marker


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


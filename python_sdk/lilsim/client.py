"""Main client class for interacting with lilsim simulator."""

import zmq
import time
import threading
from typing import Optional, Callable, Dict, Any
import logging

from . import messages_pb2

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LilsimClient:
    """Client for interacting with the lilsim simulator via ZeroMQ.
    
    This client provides interfaces for:
    - Subscribing to state updates
    - Sending control commands (async mode)
    - Responding to control requests (sync mode)
    - Sending admin commands
    - Publishing visualization markers
    
    Example usage:
        client = LilsimClient()
        client.connect()
        
        # Subscribe to state updates
        client.subscribe_state(lambda state: print(f"Car at {state.scene.car.pos.x}"))
        
        # Send async control
        client.send_control_async(steer_angle=0.1, ax=2.0)
        
        # Or register sync control callback
        def my_controller(request):
            return (0.1, 0.0, 2.0)  # (steer_angle, steer_rate, ax)
        client.register_sync_controller(my_controller)
        
        client.start()  # Start background threads
    """
    
    def __init__(self, host: str = "localhost"):
        """Initialize the client.
        
        Args:
            host: Hostname/IP of the simulator (default: localhost)
        """
        self.host = host
        self.context = zmq.Context()
        
        # Sockets
        self.state_sub: Optional[zmq.Socket] = None
        self.control_dealer: Optional[zmq.Socket] = None
        self.control_async_pub: Optional[zmq.Socket] = None
        self.admin_req: Optional[zmq.Socket] = None
        self.marker_pub: Optional[zmq.Socket] = None
        
        # State management
        self.latest_state: Optional[messages_pb2.StateUpdate] = None
        self.state_callbacks: list[Callable] = []
        self.sync_controller: Optional[Callable] = None
        
        # Threading
        self.running = False
        self.state_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None
        
        # Last control for async mode
        self.last_control = (0.0, 0.0, 0.0)  # (steer_angle, steer_rate, ax)
        
    def connect(self):
        """Connect to all simulator endpoints."""
        logger.info(f"Connecting to lilsim at {self.host}...")
        
        # State stream subscriber
        self.state_sub = self.context.socket(zmq.SUB)
        self.state_sub.connect(f"tcp://{self.host}:5556")
        self.state_sub.setsockopt(zmq.SUBSCRIBE, b"")
        logger.info("Connected to state stream (port 5556)")
        
        # Admin command requester
        self.admin_req = self.context.socket(zmq.REQ)
        self.admin_req.connect(f"tcp://{self.host}:5558")
        logger.info("Connected to admin command endpoint (port 5558)")
        
        # Async control publisher
        self.control_async_pub = self.context.socket(zmq.PUB)
        self.control_async_pub.connect(f"tcp://{self.host}:5559")
        logger.info("Connected to async control stream (port 5559)")
        
        # Marker publisher
        self.marker_pub = self.context.socket(zmq.PUB)
        self.marker_pub.connect(f"tcp://{self.host}:5560")
        logger.info("Connected to marker stream (port 5560)")
        
        # Give ZMQ time to establish connections (critical for PUB/SUB)
        time.sleep(0.5)
        
    def connect_control_sync(self):
        """Connect to synchronous control endpoint (DEALER socket).
        
        Call this only if you want to respond to control requests in sync mode.
        """
        if self.control_dealer is not None:
            logger.warning("Control sync already connected")
            return
            
        self.control_dealer = self.context.socket(zmq.DEALER)
        self.control_dealer.connect(f"tcp://{self.host}:5557")
        logger.info("Connected to sync control endpoint (port 5557)")
        
    def connect_control_async(self):
        """Connect to asynchronous control endpoint (PUB socket).
        
        This is called automatically by connect(). Use send_control_async() to send controls.
        """
        if self.control_async_pub is not None:
            logger.warning("Async control already connected")
            return
            
        self.control_async_pub = self.context.socket(zmq.PUB)
        self.control_async_pub.connect(f"tcp://{self.host}:5559")
        logger.info("Connected to async control stream (port 5559)")
        
    def subscribe_state(self, callback: Callable[[messages_pb2.StateUpdate], None]):
        """Register a callback for state updates.
        
        Args:
            callback: Function that takes a StateUpdate message
        """
        self.state_callbacks.append(callback)
        logger.info(f"Registered state callback: {callback.__name__}")
        
    def register_sync_controller(self, controller: Callable[[messages_pb2.ControlRequest], tuple[float, float, float]]):
        """Register a controller for synchronous mode.
        
        Args:
            controller: Function that takes ControlRequest and returns (steer_angle, steer_rate, ax)
                       The simulator will use the appropriate steering input based on its current mode
        """
        self.sync_controller = controller
        logger.info(f"Registered sync controller: {controller.__name__}")
        
        # Ensure control sync socket is connected
        if self.control_dealer is None:
            self.connect_control_sync()
            
    def send_control_async(self, steer_angle: float = 0.0, steer_rate: float = 0.0, ax: float = 0.0):
        """Send control command for async mode.
        
        The simulator must be in async mode for this to take effect.
        Controls are published on the async control socket (port 5559).
        The simulator will use either steer_angle or steer_rate based on its current mode.
        
        Args:
            steer_angle: Steering angle in radians (used in Angle mode)
            steer_rate: Steering rate in rad/s (used in Rate mode)
            ax: Longitudinal acceleration in m/s^2
        """
        if self.control_async_pub is None:
            logger.error("Async control socket not connected. Call connect() first.")
            return
            
        # Create and send control message
        control = messages_pb2.ControlAsync()
        control.header.version = 1
        control.steer_angle = steer_angle
        control.steer_rate = steer_rate
        control.ax = ax
        
        self.control_async_pub.send(control.SerializeToString())
        
        # Also store for sync mode fallback
        self.last_control = (steer_angle, steer_rate, ax)
        
    def _state_listener_thread(self):
        """Background thread that listens for state updates."""
        logger.info("State listener thread started")
        
        while self.running:
            try:
                # Non-blocking receive with timeout
                if self.state_sub.poll(timeout=100):  # 100ms timeout
                    msg_bytes = self.state_sub.recv()
                    state_update = messages_pb2.StateUpdate()
                    state_update.ParseFromString(msg_bytes)
                    
                    self.latest_state = state_update
                    
                    # Call all registered callbacks
                    for callback in self.state_callbacks:
                        try:
                            callback(state_update)
                        except Exception as e:
                            logger.error(f"Error in state callback: {e}")
                            
            except Exception as e:
                if self.running:
                    logger.error(f"Error in state listener: {e}")
                    
        logger.info("State listener thread stopped")
        
    def _control_responder_thread(self):
        """Background thread that responds to control requests (sync mode)."""
        logger.info("Control responder thread started")
        
        while self.running:
            try:
                # Non-blocking poll with timeout
                if self.control_dealer.poll(timeout=100):  # 100ms timeout
                    msg_bytes = self.control_dealer.recv()
                    control_request = messages_pb2.ControlRequest()
                    control_request.ParseFromString(msg_bytes)
                    
                    # Handle heartbeat probes (tick=0) - just respond immediately
                    if control_request.header.tick == 0:
                        reply = messages_pb2.ControlReply()
                        reply.header.tick = 0
                        reply.header.sim_time = 0.0
                        reply.header.version = 1
                        reply.steer_angle = 0.0
                        reply.steer_rate = 0.0
                        reply.ax = 0.0
                        self.control_dealer.send(reply.SerializeToString())
                        continue
                    
                    # Get control from controller or use last control
                    if self.sync_controller is not None:
                        try:
                            steer_angle, steer_rate, ax = self.sync_controller(control_request)
                        except Exception as e:
                            logger.error(f"Error in sync controller: {e}")
                            steer_angle, steer_rate, ax = self.last_control
                    else:
                        steer_angle, steer_rate, ax = self.last_control
                    
                    # Send reply
                    reply = messages_pb2.ControlReply()
                    reply.header.tick = control_request.header.tick
                    reply.header.sim_time = control_request.header.sim_time
                    reply.header.version = 1
                    reply.steer_angle = steer_angle
                    reply.steer_rate = steer_rate
                    reply.ax = ax
                    
                    self.control_dealer.send(reply.SerializeToString())
                    
            except Exception as e:
                if self.running:
                    logger.error(f"Error in control responder: {e}")
                    
        logger.info("Control responder thread stopped")
        
    def start(self):
        """Start background threads for listening and responding."""
        if self.running:
            logger.warning("Client already running")
            return
            
        self.running = True
        
        # Start state listener
        self.state_thread = threading.Thread(target=self._state_listener_thread, daemon=True)
        self.state_thread.start()
        
        # Start control responder if sync controller is registered
        if self.control_dealer is not None:
            self.control_thread = threading.Thread(target=self._control_responder_thread, daemon=True)
            self.control_thread.start()
            
        logger.info("Client started")
        
    def stop(self):
        """Stop all background threads."""
        if not self.running:
            return
            
        logger.info("Stopping client...")
        self.running = False
        
        if self.state_thread:
            self.state_thread.join(timeout=1.0)
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
            
        logger.info("Client stopped")
        
    def close(self):
        """Close all sockets and terminate context."""
        self.stop()
        
        if self.state_sub:
            self.state_sub.close()
        if self.control_dealer:
            self.control_dealer.close()
        if self.control_async_pub:
            self.control_async_pub.close()
        if self.admin_req:
            self.admin_req.close()
        if self.marker_pub:
            self.marker_pub.close()
            
        self.context.term()
        logger.info("Client closed")
        
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        
    # ========== Admin Commands ==========
    
    def _send_admin_command(self, cmd_type: messages_pb2.AdminCommandType, **kwargs) -> messages_pb2.AdminReply:
        """Send an admin command and wait for reply.
        
        Args:
            cmd_type: Type of admin command
            **kwargs: Additional fields for the command
            
        Returns:
            AdminReply message
        """
        cmd = messages_pb2.AdminCommand()
        cmd.header.version = 1
        cmd.type = cmd_type
        
        # Set optional fields
        if 'step_count' in kwargs:
            cmd.step_count = kwargs['step_count']
        if 'sync_mode' in kwargs:
            cmd.sync_mode = kwargs['sync_mode']
        if 'control_period_ms' in kwargs:
            cmd.control_period_ms = kwargs['control_period_ms']
        if 'params' in kwargs:
            cmd.params.CopyFrom(kwargs['params'])
            
        # Send and wait for reply
        self.admin_req.send(cmd.SerializeToString())
        reply_bytes = self.admin_req.recv()
        
        reply = messages_pb2.AdminReply()
        reply.ParseFromString(reply_bytes)
        
        if not reply.success:
            logger.warning(f"Admin command failed: {reply.message}")
        else:
            logger.info(f"Admin command succeeded: {reply.message}")
            
        return reply
        
    def pause(self) -> bool:
        """Pause the simulation.
        
        Returns:
            True if successful
        """
        reply = self._send_admin_command(messages_pb2.PAUSE)
        return reply.success
        
    def run(self) -> bool:
        """Resume/run the simulation.
        
        Returns:
            True if successful
        """
        reply = self._send_admin_command(messages_pb2.RUN)
        return reply.success
        
    def reset(self) -> bool:
        """Reset the simulation.
        
        Returns:
            True if successful
        """
        reply = self._send_admin_command(messages_pb2.RESET)
        return reply.success
        
    def step(self, count: int = 1) -> bool:
        """Step the simulation by N ticks.
        
        Args:
            count: Number of ticks to step
            
        Returns:
            True if successful
        """
        reply = self._send_admin_command(messages_pb2.STEP, step_count=count)
        return reply.success
        
    def set_mode(self, sync: bool, control_period_ms: int = 100) -> bool:
        """Set synchronous or asynchronous mode.
        
        Args:
            sync: True for sync mode, False for async
            control_period_ms: Milliseconds between control requests (sync mode only)
            
        Returns:
            True if successful
        """
        reply = self._send_admin_command(
            messages_pb2.SET_MODE,
            sync_mode=sync,
            control_period_ms=control_period_ms
        )
        return reply.success
        
    def set_params(self, dt: float = None, wheelbase: float = None, 
                   v_max: float = None, delta_max: float = None, Lf: float = None,
                   ax_max: float = None, steer_rate_max: float = None,
                   steering_mode: str = None) -> bool:
        """Set simulation parameters.
        
        Args:
            dt: Timestep in seconds
            wheelbase: Wheelbase in meters
            v_max: Maximum velocity in m/s
            delta_max: Maximum steering angle in radians
            Lf: Distance from CG to front axle in meters
            ax_max: Maximum acceleration in m/s^2
            steer_rate_max: Maximum steering rate in rad/s
            steering_mode: 'angle' or 'rate' (None to keep current)
            
        Returns:
            True if successful
        """
        params = messages_pb2.SimParams()
        if dt is not None:
            params.dt = dt
        if wheelbase is not None:
            params.wheelbase = wheelbase
        if v_max is not None:
            params.v_max = v_max
        if delta_max is not None:
            params.delta_max = delta_max
        if Lf is not None:
            params.Lf = Lf
        if ax_max is not None:
            params.ax_max = ax_max
        if steer_rate_max is not None:
            params.steer_rate_max = steer_rate_max
        if steering_mode is not None:
            if steering_mode.lower() == 'angle':
                params.steering_mode = messages_pb2.ANGLE
            elif steering_mode.lower() == 'rate':
                params.steering_mode = messages_pb2.RATE
            else:
                logger.warning(f"Invalid steering_mode '{steering_mode}', ignoring")
            
        reply = self._send_admin_command(messages_pb2.SET_PARAMS, params=params)
        return reply.success
        
    # ========== Marker Publishing ==========
    
    def publish_marker(self, ns: str, id: int, marker_type: messages_pb2.MarkerType,
                      x: float = 0, y: float = 0, yaw: float = 0,
                      r: int = 255, g: int = 255, b: int = 255, a: int = 255,
                      scale_x: float = 1.0, scale_y: float = 1.0,
                      text: str = "", points: list = None,
                      ttl_sec: float = 0.0, visible: bool = True):
        """Publish a single marker.
        
        Args:
            ns: Namespace for the marker
            id: Unique ID within namespace
            marker_type: Type of marker (from MarkerType enum)
            x, y, yaw: Pose of the marker
            r, g, b, a: Color (0-255)
            scale_x, scale_y: Scale factors
            text: Text content (for TEXT markers)
            points: List of (x, y, yaw) tuples (for LINE_STRIP, etc.)
            ttl_sec: Time-to-live in seconds (0 = infinite)
            visible: Visibility flag
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = marker_type
        marker.pose.x = x
        marker.pose.y = y
        marker.pose.yaw = yaw
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.text = text
        marker.ttl_sec = ttl_sec
        marker.visible = visible
        
        if points:
            for px, py, pyaw in points:
                pt = marker.points.add()
                pt.x = px
                pt.y = py
                pt.yaw = pyaw
                
        # Wrap in MarkerArray
        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])
        
    def publish_markers(self, markers: list):
        """Publish multiple markers at once.
        
        Args:
            markers: List of Marker protobuf messages
        """
        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.extend(markers)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])
    
    def delete_marker(self, ns: str, marker_id: int):
        """Delete a specific marker.
        
        Args:
            ns: Namespace of the marker
            marker_id: ID of the marker within the namespace
        """
        cmd = messages_pb2.MarkerCommand()
        cmd.header.version = 1
        cmd.type = messages_pb2.DELETE_MARKER
        cmd.ns = ns
        cmd.id = marker_id
        
        self.marker_pub.send_multipart([b"COMMAND", cmd.SerializeToString()])
    
    def delete_namespace(self, ns: str):
        """Delete all markers in a namespace.
        
        Args:
            ns: Namespace to delete
        """
        cmd = messages_pb2.MarkerCommand()
        cmd.header.version = 1
        cmd.type = messages_pb2.DELETE_NAMESPACE
        cmd.ns = ns
        
        self.marker_pub.send_multipart([b"COMMAND", cmd.SerializeToString()])
    
    def clear_markers(self):
        """Clear all markers from the visualization."""
        cmd = messages_pb2.MarkerCommand()
        cmd.header.version = 1
        cmd.type = messages_pb2.CLEAR_ALL
        
        self.marker_pub.send_multipart([b"COMMAND", cmd.SerializeToString()])
        
    # ========== Convenience methods ==========
    
    def get_latest_state(self) -> Optional[messages_pb2.StateUpdate]:
        """Get the most recent state update.
        
        Returns:
            Latest StateUpdate or None if no state received yet
        """
        return self.latest_state
        
    def wait_for_state(self, timeout: float = 5.0) -> Optional[messages_pb2.StateUpdate]:
        """Wait for the first state update.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            StateUpdate or None if timeout
        """
        start = time.time()
        while self.latest_state is None and (time.time() - start) < timeout:
            time.sleep(0.01)
        return self.latest_state


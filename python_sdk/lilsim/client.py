"""Main client class for interacting with lilsim simulator."""

import logging
import time
import threading
from typing import Any, Callable, Dict, Mapping, Optional, Sequence

import numpy as np
import zmq

from . import messages_pb2

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LilsimClient:
    """ZeroMQ client that mirrors the simulator's metadata-driven API."""
    
    def __init__(self, host: str = "localhost"):
        """Initialize sockets and metadata caches."""
        self.host = host
        self.context = zmq.Context()
        
        # Sockets
        self.state_sub: Optional[zmq.Socket] = None
        self.control_dealer: Optional[zmq.Socket] = None
        self.control_async_pub: Optional[zmq.Socket] = None
        self.admin_req: Optional[zmq.Socket] = None
        self.marker_pub: Optional[zmq.Socket] = None
        
        # Metadata caches
        self.metadata: Optional[messages_pb2.ModelMetadata] = None
        self.metadata_version: int = 0
        self.param_name_to_index: Dict[str, int] = {}
        self.setting_name_to_index: Dict[str, int] = {}
        self.input_name_to_index: Dict[str, int] = {}
        self._zero_input_vector: list[float] = []
        self.last_control_vector: list[float] = []
        
        # State management
        self.latest_state: Optional[messages_pb2.StateUpdate] = None
        self.state_callbacks: list[Callable] = []
        self.sync_controller: Optional[Callable] = None
        
        # Threading
        self.running = False
        self.state_thread: Optional[threading.Thread] = None
        self.control_thread: Optional[threading.Thread] = None
        
        # Last control for sync fallback
        self.last_control = (0.0, 0.0, 0.0)
        
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
        
    def refresh_metadata(self) -> messages_pb2.ModelMetadata:
        """Fetch the latest ModelMetadata via GET_METADATA."""
        reply = self._send_admin_command(messages_pb2.GET_METADATA)
        if not reply.success or not reply.HasField("metadata"):
            raise RuntimeError(f"Failed to fetch metadata: {reply.message}")
        self._apply_metadata(reply.metadata)
        return self.metadata
    
    def get_metadata(self, refresh: bool = False) -> Optional[messages_pb2.ModelMetadata]:
        """Return cached metadata (optionally refreshing first)."""
        if refresh or self.metadata is None:
            self.refresh_metadata()
        return self.metadata
        
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
        
    def register_sync_controller(
        self,
        controller: Callable[[messages_pb2.ControlRequest, Dict[str, float]], Mapping[str, float] | Sequence[float]],
    ) -> None:
        """Register a synchronous controller callback.
        
        The callback receives the raw ``ControlRequest`` message and a convenience
        dictionary produced by :meth:`decode_scene_state`. It must return either:

        - A mapping of ``input_name -> value``
        - A full input vector ordered exactly like ``ModelMetadata.inputs``
        - A legacy ``(steer_angle, steer_rate, ax)`` tuple
        """
        self.sync_controller = controller
        logger.info("Registered sync controller: %s", getattr(controller, "__name__", repr(controller)))
        
        # Ensure control sync socket is connected
        if self.control_dealer is None:
            self.connect_control_sync()
            
    def send_control_async(
        self,
        overrides: Optional[Mapping[str, float]] = None,
        **legacy_inputs: float,
    ) -> None:
        """Publish asynchronous control values by input name.
        
        Args:
            overrides: Mapping of input name -> value.
            **legacy_inputs: Optional steer_angle/steer_rate/ax convenience kwargs.
        """
        payload: Dict[str, float] = {}
        if overrides:
            payload.update(overrides)
        payload.update(self._legacy_control_args(**legacy_inputs))
        if not payload:
            raise ValueError("At least one control input must be provided.")
        self.send_control_async_named(payload)
    
    def send_control_async_named(self, inputs: Mapping[str, float]) -> None:
        """Publish asynchronous control values using a name/value mapping."""
        vector = self._build_input_vector(inputs)
        self.send_control_async_vector(vector)
    
    def send_control_async_vector(self, values: Sequence[float]) -> None:
        """Publish an already ordered control vector matching metadata."""
        self._ensure_metadata()
        if self.control_async_pub is None:
            logger.error("Async control socket not connected. Call connect() first.")
            return
        if len(values) != len(self._zero_input_vector):
            raise ValueError("Control vector length does not match metadata inputs.")

        msg = messages_pb2.ControlAsync()
        msg.header.version = 1
        msg.metadata_version = self.metadata_version
        msg.input_values.extend(float(v) for v in values)

        self.control_async_pub.send(msg.SerializeToString())
        self.last_control_vector = [float(v) for v in values]
        
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
                if not self.control_dealer.poll(timeout=100):
                    continue
                msg_bytes = self.control_dealer.recv()
                control_request = messages_pb2.ControlRequest()
                control_request.ParseFromString(msg_bytes)

                scene_meta_version = control_request.scene.metadata_version
                if scene_meta_version and scene_meta_version != self.metadata_version:
                    try:
                        self.refresh_metadata()
                    except RuntimeError as exc:
                        logger.warning("Failed to refresh metadata after mismatch: %s", exc)
                version_for_reply = control_request.scene.metadata_version or self.metadata_version

                reply = messages_pb2.ControlReply()
                reply.header.CopyFrom(control_request.header)
                reply.metadata_version = version_for_reply

                # Heartbeat probe
                if control_request.header.tick == 0:
                    reply.input_values.extend(self._zero_input_vector)
                    self.control_dealer.send(reply.SerializeToString())
                    continue

                self._ensure_metadata()
                vector: Optional[list[float]] = None
                if self.sync_controller is not None:
                    try:
                        state_dict = self.decode_scene_state(control_request.scene)
                        control_output = self.sync_controller(control_request, state_dict)
                        vector = self._format_control_output(control_output)
                    except Exception as exc:
                        logger.error("Error in sync controller: %s", exc)
                        vector = None

                if vector is None:
                    if not self.last_control_vector:
                        self.last_control_vector = list(self._zero_input_vector)
                    vector = list(self.last_control_vector)
                else:
                    self.last_control_vector = list(vector)

                reply.input_values.extend(vector)
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
    
    def _send_admin_command(
        self,
        cmd_type: messages_pb2.AdminCommandType,
        builder: Optional[Callable[[messages_pb2.AdminCommand], None]] = None,
        **kwargs,
    ) -> messages_pb2.AdminReply:
        """Send an admin command and wait for the reply."""
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
        if 'track_path' in kwargs:
            cmd.track_path = kwargs['track_path']
        if builder:
            builder(cmd)
            
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
        """Pause the simulation."""
        reply = self._send_admin_command(messages_pb2.PAUSE)
        return reply.success
        
    def run(self) -> bool:
        """Resume the simulation."""
        reply = self._send_admin_command(messages_pb2.RUN)
        return reply.success
        
    def reset(self) -> bool:
        """Trigger a simulator reset."""
        reply = self._send_admin_command(messages_pb2.RESET)
        return reply.success
        
    def step(self, count: int = 1) -> bool:
        """Advance the simulation by ``count`` ticks."""
        reply = self._send_admin_command(messages_pb2.STEP, step_count=count)
        return reply.success
        
    def set_control_mode(self, sync: bool, control_period_ms: int = 100, external_control: bool = True) -> bool:
        """Configure synchronous/asynchronous mode and preferred control source."""
        def builder(cmd: messages_pb2.AdminCommand) -> None:
            cmd.sync_mode = sync
            cmd.control_period_ms = control_period_ms
            cmd.use_external_control = external_control

        reply = self._send_admin_command(messages_pb2.SET_CONTROL_MODE, builder=builder)
        return reply.success
    
    def set_simulation_config(self,
                              timestep: Optional[float] = None,
                              run_speed: Optional[float] = None) -> bool:
        """Set simulator-level configuration such as timestep and run speed."""
        if timestep is None and run_speed is None:
            raise ValueError("Provide at least one of timestep or run_speed.")

        def builder(cmd: messages_pb2.AdminCommand) -> None:
            if timestep is not None:
                cmd.timestep = timestep
            if run_speed is not None:
                cmd.run_speed = run_speed

        reply = self._send_admin_command(messages_pb2.SET_SIM_CONFIG, builder=builder)
        return reply.success

    def get_simulation_config(self) -> tuple[Optional[float], Optional[float]]:
        """Fetch the currently requested timestep and run speed."""
        reply = self._send_admin_command(messages_pb2.GET_SIM_CONFIG)
        if not reply.success:
            raise RuntimeError(f"Failed to fetch simulation config: {reply.message}")
        timestep = reply.timestep if reply.HasField("timestep") else None
        run_speed = reply.run_speed if reply.HasField("run_speed") else None
        return timestep, run_speed
        
    def set_parameters(self, overrides: Mapping[str, float]) -> bool:
        """Set parameter values by name using ModelMetadata indices."""
        updates = self._build_param_updates(overrides)
        if not updates:
            logger.warning("No valid parameter overrides supplied.")
            return False

        def builder(cmd: messages_pb2.AdminCommand) -> None:
            for index, value in updates:
                entry = cmd.param_updates.add()
                entry.index = index
                entry.value = value

        reply = self._send_admin_command(messages_pb2.SET_PARAMS, builder=builder)
        return reply.success
    
    def set_settings(self, overrides: Mapping[str, int]) -> bool:
        """Set setting values by name using ModelMetadata indices."""
        updates = self._build_setting_updates(overrides)
        if not updates:
            logger.warning("No valid setting overrides supplied.")
            return False

        def builder(cmd: messages_pb2.AdminCommand) -> None:
            for index, value in updates:
                entry = cmd.setting_updates.add()
                entry.index = index
                entry.value = value

        reply = self._send_admin_command(messages_pb2.SET_SETTINGS, builder=builder)
        return reply.success
    
    def set_track(self, track_path: str) -> bool:
        """Load a track from a CSV file.
        
        Args:
            track_path: Full or relative path to the track CSV file
            
        Returns:
            True if successful
        """
        reply = self._send_admin_command(messages_pb2.SET_TRACK, track_path=track_path)
        return reply.success
    
    def load_param_profile(self, path: str) -> bool:
        """Load a YAML parameter profile."""
        if not path:
            raise ValueError("Parameter profile path must be provided.")

        def builder(cmd: messages_pb2.AdminCommand) -> None:
            cmd.param_profile_path = path

        reply = self._send_admin_command(messages_pb2.LOAD_PARAM_PROFILE, builder=builder)
        return reply.success
    
    def clear_param_profile(self) -> bool:
        """Clear the currently staged parameter profile."""
        reply = self._send_admin_command(messages_pb2.CLEAR_PARAM_PROFILE)
        return reply.success
        
    # ========== Marker Publishing ==========
    
    def publish_marker(self, ns: str, id: int, 
                      frame_id: messages_pb2.FrameId = None,
                      marker_type: messages_pb2.MarkerType = None,
                      x: float = 0, y: float = 0, yaw: float = 0,
                      r: int = 255, g: int = 255, b: int = 255, a: int = 255,
                      scale_x: float = 1.0, scale_y: float = 1.0,
                      text: str = "", 
                      points: list | np.ndarray = None,
                      colors: list | np.ndarray = None,
                      ttl_sec: float = 0.0, visible: bool = True):
        """Publish a single marker.
        
        Args:
            ns: Namespace for the marker
            id: Unique ID within namespace
            frame_id: Reference frame (WORLD or CAR)
            marker_type: Type of marker (from MarkerType enum)
            x, y, yaw: Pose of the marker
            r, g, b, a: Color (0-255)
            scale_x, scale_y: Scale factors
            text: Text content (for TEXT markers)
            points: List of (x, y) tuples (for LINE_STRIP, etc.)
            colors: List of (r, g, b, a) tuples for per-vertex colors
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
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD
        
        if points is not None:
            for point in points:
                pt = marker.points.add()
                if len(point) >= 2:
                    pt.x = point[0]
                    pt.y = point[1]
                else:
                    pt.x = point[0] if len(point) > 0 else 0
                    pt.y = 0
        
        if colors is not None:
            for color in colors:
                c = marker.colors.add()
                c.r = color[0] if len(color) > 0 else 255
                c.g = color[1] if len(color) > 1 else 255
                c.b = color[2] if len(color) > 2 else 255
                c.a = color[3] if len(color) > 3 else 255
                
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

    def publish_line_strip(self, ns: str, id: int, 
                          frame_id: messages_pb2.FrameId = None,
                          points: list | np.ndarray = None,
                          colors: list = None,
                          color: tuple = (255, 0, 0, 255),
                          line_width: float = 0.05,
                          ttl_sec: float = 0.0):
        """Publish a line strip marker.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            points: List of (x, y) tuples
            colors: Optional list of RGBA tuples for per-vertex colors
            color: RGBA tuple (0-255) - used if colors not provided
            line_width: Line width in meters
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.LINE_STRIP
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.scale.x = line_width
        marker.scale.y = line_width
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        for point in points:
            pt = marker.points.add()
            pt.x = point[0]
            pt.y = point[1]
        
        if colors is not None:
            for c in colors:
                col = marker.colors.add()
                col.r = c[0]
                col.g = c[1]
                col.b = c[2]
                col.a = c[3] if len(c) > 3 else 255
            
        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_circle(self, ns: str, id: int, 
                      frame_id: messages_pb2.FrameId = None,
                      pos: tuple[float, float] = (0, 0),
                      radius: float = 0.5,
                      color: tuple = (255, 0, 0, 255),
                      ttl_sec: float = 0.0):
        """Publish a circle marker.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            pos: Tuple of (x, y) position
            radius: Circle radius in meters
            color: RGBA tuple (0-255)
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.CIRCLE
        marker.pose.x = pos[0]
        marker.pose.y = pos[1]
        marker.pose.yaw = 0.0
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_text(self, ns: str, id: int, 
                     frame_id: messages_pb2.FrameId = None,
                     pos: tuple[float, float] = (0, 0), 
                     text: str = "",
                     color: tuple = (255, 255, 255, 255),
                     scale: float = 1.0,
                     ttl_sec: float = 0.0):
        """Publish a text marker.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            pos: Tuple of (x, y) position
            text: Text content
            color: RGBA tuple (0-255)
            scale: Text scale
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.TEXT
        marker.pose.x = pos[0]
        marker.pose.y = pos[1]
        marker.text = text
        marker.scale.x = scale
        marker.scale.y = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)

        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_arrow(self, ns: str, id: int, 
                      frame_id: messages_pb2.FrameId = None,
                      from_pos: tuple[float, float] = (0, 0), 
                      to_pos: tuple[float, float] = (1, 0),
                      thickness: float = 0.1,
                      color: tuple = (255, 255, 255, 255),
                      ttl_sec: float = 0.0):
        """Publish an arrow marker.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            from_pos: Tuple of (x, y) from position
            to_pos: Tuple of (x, y) to position
            thickness: Arrow shaft/head thickness in meters
            color: RGBA tuple (0-255)
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.ARROW
        marker.pose.x = from_pos[0]
        marker.pose.y = from_pos[1]
        
        # Calculate direction and length
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        length = np.sqrt(dx * dx + dy * dy)
        marker.pose.yaw = np.arctan2(dy, dx)
        
        marker.scale.x = length
        marker.scale.y = thickness
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)

        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_ring(self, ns: str, id: int, 
                     frame_id: messages_pb2.FrameId = None,
                     pos: tuple[float, float] = (0, 0),
                     radius: float = 0.5,
                     line_width: float = 0.05,
                     color: tuple = (255, 255, 255, 255),
                     num_segments: int = 32,
                     ttl_sec: float = 0.0):
        """Publish a ring marker (circle outline using LINE_STRIP).
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            pos: Tuple of (x, y) position
            radius: Ring radius in meters
            line_width: Line width in meters
            color: RGBA tuple (0-255)
            num_segments: Number of line segments to approximate circle
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        # Generate circle points
        points = []
        for i in range(num_segments + 1):  # +1 to close the circle
            angle = 2 * np.pi * i / num_segments
            x = pos[0] + radius * np.cos(angle)
            y = pos[1] + radius * np.sin(angle)
            points.append((x, y))
        
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.LINE_STRIP
        marker.scale.x = line_width
        marker.scale.y = line_width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        for point in points:
            pt = marker.points.add()
            pt.x = point[0]
            pt.y = point[1]

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)

        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_rectangle(self, ns: str, id: int, 
                         frame_id: messages_pb2.FrameId = None,
                         pos: tuple[float, float] = (0, 0),
                         width: float = 1.0, 
                         height: float = 1.0,
                         yaw: float = 0.0,
                         color: tuple = (255, 255, 255, 255),
                         ttl_sec: float = 0.0):
        """Publish a rectangle marker.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            pos: Tuple of (x, y) position (center)
            width: Rectangle width in meters
            height: Rectangle height in meters
            yaw: Rotation angle in radians
            color: RGBA tuple (0-255)
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.RECTANGLE
        marker.pose.x = pos[0]
        marker.pose.y = pos[1]
        marker.pose.yaw = yaw
        marker.scale.x = width
        marker.scale.y = height
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_circle_list(self, ns: str, id: int, 
                           frame_id: messages_pb2.FrameId = None,
                           positions: list | np.ndarray = None,
                           colors: list = None,
                           radius: float = 0.5,
                           color: tuple = (255, 255, 255, 255),
                           ttl_sec: float = 0.0):
        """Publish a list of circles.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            positions: List of (x, y) tuples
            colors: Optional list of RGBA tuples for per-circle colors
            radius: Radius for all circles in meters
            color: RGBA tuple (0-255) - used if colors not provided
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.CIRCLE_LIST
        marker.scale.x = radius * 2  # diameter
        marker.scale.y = radius * 2
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        for pos in positions:
            pt = marker.points.add()
            pt.x = pos[0]
            pt.y = pos[1]
        
        if colors is not None:
            for c in colors:
                col = marker.colors.add()
                col.r = c[0]
                col.g = c[1]
                col.b = c[2]
                col.a = c[3] if len(c) > 3 else 255

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_triangle_list(self, ns: str, id: int, 
                             frame_id: messages_pb2.FrameId = None,
                             triangles: list | np.ndarray = None,
                             colors: list = None,
                             color: tuple = (255, 255, 255, 255),
                             ttl_sec: float = 0.0):
        """Publish a list of triangles.
        
        Args:
            ns: Namespace
            id: Marker ID
            frame_id: Reference frame (WORLD or CAR)
            triangles: List of (x, y) tuples (every 3 points forms a triangle)
            colors: Optional list of RGBA tuples for per-vertex colors
            color: RGBA tuple (0-255) - used if colors not provided
            ttl_sec: Time-to-live in seconds (0 = infinite)
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.TRIANGLE_LIST
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.ttl_sec = ttl_sec
        marker.visible = True
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD

        for vertex in triangles:
            pt = marker.points.add()
            pt.x = vertex[0]
            pt.y = vertex[1]
        
        if colors is not None:
            for c in colors:
                col = marker.colors.add()
                col.r = c[0]
                col.g = c[1]
                col.b = c[2]
                col.a = c[3] if len(c) > 3 else 255

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)
        
        self.marker_pub.send_multipart([b"MARKERS", array.SerializeToString()])

    def publish_car_marker(self, ns: str, id: int,
                           pose: tuple[float, float, float],
                           wheelbase: float,
                           track_width: float,
                           wheel_fl_angle: float | None = None,
                           wheel_fr_angle: float | None = None,
                           opacity: float = 1.0,
                           tint_color: tuple[int, int, int, int] | None = None,
                           tint_opacity: float = 0.0,
                           frame_id: messages_pb2.FrameId = None,
                           ttl_sec: float = 0.0):
        """Publish a car sprite marker that reuses the simulator's car textures.

        Args:
            ns: Marker namespace.
            id: Marker id within the namespace.
            pose: Tuple of (x, y, yaw) in world coordinates.
            wheelbase: Wheelbase in meters for scaling the sprite.
            track_width: Track width in meters for scaling the sprite.
            wheel_fl_angle: Optional front-left wheel angle in radians.
            wheel_fr_angle: Optional front-right wheel angle in radians.
            opacity: Overall sprite opacity [0, 1].
            tint_color: Optional RGBA tuple (0-255) used for highlighting.
            tint_opacity: Tint overlay opacity [0, 1].
            frame_id: Frame the pose is expressed in (defaults to WORLD).
            ttl_sec: Optional lifetime (0 = infinite).
        """
        marker = messages_pb2.Marker()
        marker.ns = ns
        marker.id = id
        marker.type = messages_pb2.CAR_SPRITE
        marker.pose.x = pose[0]
        marker.pose.y = pose[1]
        marker.pose.yaw = pose[2]
        marker.frame_id = frame_id if frame_id is not None else messages_pb2.WORLD
        marker.ttl_sec = ttl_sec
        marker.visible = True

        car = marker.car
        car.wheelbase = float(wheelbase)
        car.track_width = float(track_width)
        if wheel_fl_angle is not None:
            car.wheel_fl_angle = float(wheel_fl_angle)
        if wheel_fr_angle is not None:
            car.wheel_fr_angle = float(wheel_fr_angle)
        car.opacity = float(np.clip(opacity, 0.0, 1.0))
        car.tint_opacity = float(np.clip(tint_opacity, 0.0, 1.0))

        if tint_color is not None:
            marker.color.r = int(np.clip(tint_color[0], 0, 255))
            marker.color.g = int(np.clip(tint_color[1], 0, 255))
            marker.color.b = int(np.clip(tint_color[2], 0, 255))
            marker.color.a = int(np.clip(tint_color[3], 0, 255))
        else:
            marker.color.r = 255
            marker.color.g = 255
            marker.color.b = 255
            marker.color.a = 255

        array = messages_pb2.MarkerArray()
        array.header.version = 1
        array.markers.append(marker)

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
    
    # ========== Metadata + control helpers ==========
    
    def _apply_metadata(self, metadata: messages_pb2.ModelMetadata) -> None:
        """Cache metadata and rebuild name -> index maps."""
        self.metadata = metadata
        self.metadata_version = metadata.schema_version
        self.param_name_to_index = {entry.name: idx for idx, entry in enumerate(metadata.params)}
        self.setting_name_to_index = {entry.name: idx for idx, entry in enumerate(metadata.settings)}
        self.input_name_to_index = {entry.name: idx for idx, entry in enumerate(metadata.inputs)}
        self._zero_input_vector = [0.0] * len(metadata.inputs)
        self.last_control_vector = list(self._zero_input_vector)
    
    def _ensure_metadata(self) -> None:
        """Ensure metadata is available before using name-based helpers."""
        if self.metadata is None:
            self.refresh_metadata()
    
    def _build_param_updates(self, overrides: Mapping[str, float]) -> list[tuple[int, float]]:
        """Convert name/value dict to list of (index, value)."""
        self._ensure_metadata()
        updates: list[tuple[int, float]] = []
        for name, value in overrides.items():
            idx = self.param_name_to_index.get(name)
            if idx is None:
                logger.warning("Unknown parameter '%s'; skipping.", name)
                continue
            updates.append((idx, float(value)))
        return updates
    
    def _build_setting_updates(self, overrides: Mapping[str, int]) -> list[tuple[int, int]]:
        """Convert name/value dict to list of (index, value)."""
        self._ensure_metadata()
        updates: list[tuple[int, int]] = []
        for name, value in overrides.items():
            idx = self.setting_name_to_index.get(name)
            if idx is None:
                logger.warning("Unknown setting '%s'; skipping.", name)
                continue
            updates.append((idx, int(value)))
        return updates
    
    def _build_input_vector(self, overrides: Mapping[str, float]) -> list[float]:
        """Return a full input vector populated from the provided overrides."""
        self._ensure_metadata()
        vector = list(self._zero_input_vector)
        for name, value in overrides.items():
            idx = self.input_name_to_index.get(name)
            if idx is None:
                logger.warning("Unknown input '%s'; skipping.", name)
                continue
            vector[idx] = float(value)
        return vector
    
    def _resolve_input_name(self, preferred: str) -> Optional[str]:
        """Resolve a canonical input name, falling back to substring matches."""
        self._ensure_metadata()
        if preferred in self.input_name_to_index:
            return preferred
        for name in self.input_name_to_index:
            if preferred in name:
                return name
        logger.warning("Unable to resolve input '%s' from metadata.", preferred)
        return None
    
    def _legacy_control_args(
        self,
        steer_angle: Optional[float] = None,
        steer_rate: Optional[float] = None,
        ax: Optional[float] = None,
    ) -> Dict[str, float]:
        """Translate legacy keyword args into metadata input names."""
        overrides: Dict[str, float] = {}
        if steer_angle is not None:
            name = self._resolve_input_name("steering_wheel_angle_input")
            if name:
                overrides[name] = steer_angle
        if steer_rate is not None:
            name = self._resolve_input_name("steering_wheel_rate_input")
            if name:
                overrides[name] = steer_rate
        if ax is not None:
            name = self._resolve_input_name("ax_input")
            if name:
                overrides[name] = ax
        return overrides
    
    def _format_control_output(self, output: Any) -> Optional[list[float]]:
        """Coerce controller output into a metadata-ordered vector."""
        if output is None:
            return None
        if isinstance(output, Mapping):
            return self._build_input_vector(output)
        if isinstance(output, Sequence) and not isinstance(output, (str, bytes, bytearray)):
            if len(output) == len(self._zero_input_vector):
                return [float(v) for v in output]
            if len(output) == 3:
                overrides = self._legacy_control_args(
                    steer_angle=output[0],
                    steer_rate=output[1],
                    ax=output[2],
                )
                return self._build_input_vector(overrides)
        return None
    
    def decode_scene_state(self, scene: messages_pb2.SceneState) -> Dict[str, Any]:
        """Convert a SceneState into dict form using cached metadata."""
        self._ensure_metadata()
        data: Dict[str, Any] = {
            "tick": scene.header.tick,
            "sim_time": scene.header.sim_time,
            "states": {},
            "inputs": {},
            "params": {},
            "settings": {},
        }
        if self.metadata:
            for idx, value in enumerate(scene.state_values):
                name = self.metadata.states[idx].name if idx < len(self.metadata.states) else f"state_{idx}"
                data["states"][name] = value
            for idx, value in enumerate(scene.input_values):
                name = self.metadata.inputs[idx].name if idx < len(self.metadata.inputs) else f"input_{idx}"
                data["inputs"][name] = value
            for idx, value in enumerate(scene.param_values):
                name = self.metadata.params[idx].name if idx < len(self.metadata.params) else f"param_{idx}"
                data["params"][name] = value
            for idx, value in enumerate(scene.setting_values):
                name = self.metadata.settings[idx].name if idx < len(self.metadata.settings) else f"setting_{idx}"
                data["settings"][name] = int(value)
        return data
    
    def decode_state_update(self, update: messages_pb2.StateUpdate) -> Dict[str, Any]:
        """Decode a StateUpdate helper wrapper."""
        return self.decode_scene_state(update.scene)
        
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

    def get_car_parameter(self, name: str) -> float:
        """Return the currently staged value for a car parameter (from metadata)."""
        metadata = self.get_metadata()
        for meta in metadata.params:
            if meta.name == name:
                return meta.default_value
        raise KeyError(f"Unknown parameter '{name}'. Available: {[p.name for p in metadata.params]}")

    def get_car_setting(self, name: str) -> int:
        """Return the currently staged index for a setting (from metadata)."""
        metadata = self.get_metadata()
        for meta in metadata.settings:
            if meta.name == name:
                return meta.default_index
        raise KeyError(f"Unknown setting '{name}'. Available: {[s.name for s in metadata.settings]}")

    def get_metadata_summary(self, refresh: bool = False) -> Dict[str, Any]:
        """Return a dictionary with easy-to-query metadata contents."""
        metadata = self.get_metadata(refresh=refresh)
        summary: Dict[str, Any] = {
            "model_name": metadata.model_name,
            "schema_version": metadata.schema_version,
            "params": {},
            "settings": {},
            "inputs": {},
            "states": {},
        }

        for meta in metadata.params:
            limits = meta.limits
            summary["params"][meta.name] = {
                "value": meta.default_value,
                "min": limits.min if limits is not None else None,
                "max": limits.max if limits is not None else None,
            }
        for meta in metadata.settings:
            summary["settings"][meta.name] = {
                "value": meta.default_index,
                "options": list(meta.options),
            }
        for meta in metadata.inputs:
            limits = meta.limits
            summary["inputs"][meta.name] = {
                "min": limits.min if limits is not None else None,
                "max": limits.max if limits is not None else None,
            }
        for meta in metadata.states:
            limits = meta.limits
            summary["states"][meta.name] = {
                "min": limits.min if limits is not None else None,
                "max": limits.max if limits is not None else None,
            }
        return summary


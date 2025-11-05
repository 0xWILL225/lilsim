# Communication Module (ZeroMQ)

The `comm` module provides ZeroMQ-based communication between the lilsim simulator and external clients (e.g., Jupyter notebooks, Python scripts).

## Architecture

### Socket Layout

| Socket | Direction | Pattern | Port | Message Types | Purpose |
|--------|-----------|---------|------|---------------|---------|
| **State Stream** | sim → client | PUB/SUB | 5556 | `StateUpdate` | Broadcasts simulation state every tick |
| **Synchronous Control** | sim ↔ client | REQ/REP | 5557 | `ControlRequest`/`ControlReply` | Requests control input in sync mode |
| **Admin Commands** | client ↔ sim | REQ/REP | 5558 | `AdminCommand`/`AdminReply` | Remote control (pause, reset, params, etc.) |
| **Marker Stream** | client → viz | PUB/SUB | 5560 | `MarkerArray` | Visualization markers from clients |

## Usage

### Enabling Communication

In the GUI:
1. Open the "Communication" section in the Admin panel
2. Toggle "Enable ZMQ" to start the communication server
3. Select "Synchronous" or "Asynchronous" mode
4. Configure control period (K ticks) for synchronous mode

### Operating Modes

#### Asynchronous Mode (Default)
- Simulator runs continuously without waiting for client
- Uses latest control input received from client
- Sample-and-hold: maintains last control until new one arrives
- Non-blocking, suitable for real-time visualization

#### Synchronous Mode
- Simulator pauses every K ticks to request control
- Waits for client response with configurable timeout (default: 100ms)
- On timeout: applies "hold" policy (uses last control)
- Deterministic, ideal for step-through debugging in Jupyter

### Message Format

All messages use Protocol Buffers (see `messages.proto`). Key types:

- **StateUpdate**: Contains car position, velocity, yaw, yaw_rate
- **ControlRequest**: Includes current state and simulation tick
- **ControlReply**: Steering angle (rad) and longitudinal acceleration (m/s²)
- **AdminCommand**: Commands like PAUSE, RUN, STEP, RESET, SET_PARAMS, SET_MODE
- **MarkerArray**: Visualization markers (lines, circles, etc.) with TTL support

### Client Example (Python)

```python
import zmq
import messages_pb2  # Generated from messages.proto

# Subscribe to state updates
context = zmq.Context()
state_sub = context.socket(zmq.SUB)
state_sub.connect("tcp://localhost:5556")
state_sub.setsockopt(zmq.SUBSCRIBE, b"")

# Send control (for async mode - not shown here)
# Or respond to control requests (for sync mode - not shown here)

# Receive state updates
while True:
    msg_bytes = state_sub.recv()
    state_update = messages_pb2.StateUpdate()
    state_update.ParseFromString(msg_bytes)
    
    car = state_update.scene.car
    print(f"Tick: {state_update.scene.header.tick}")
    print(f"Position: ({car.pos.x}, {car.pos.y})")
    print(f"Velocity: {car.v} m/s")
```

## Implementation Details

### CommServer (sim/Simulator)
- Manages 3 sockets: state PUB, control REQ, admin REP
- State publishing happens every tick (non-blocking)
- Control requests are synchronous with timeout
- Admin commands are polled and handled non-blocking

### MarkerSubscriber (viz/Application)
- Subscribes to marker messages from clients
- Automatically converts protobuf markers to internal format
- Integrates with existing MarkerSystem for display and visibility

### Thread Safety
- All communication is thread-safe
- Simulator runs in separate thread
- Atomic flags for mode switching and enable/disable

## Dependencies

- **ZeroMQ**: Message transport (via vcpkg)
- **cppzmq**: C++ bindings for ZeroMQ
- **Protocol Buffers**: Message serialization
- **spdlog**: Logging

## Notes

- All endpoints are fixed (no dynamic port allocation)
- Protocol version is included in all message headers
- Simulation time is tracked in seconds (tick * dt)
- Markers support TTL (time-to-live) based on simulation time
- TTL does not elapse when simulation is paused


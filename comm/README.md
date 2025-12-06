# Communication Module (ZeroMQ)

The `comm` module provides ZeroMQ-based communication between the lilsim simulator and external clients (e.g., Jupyter notebooks, Python scripts).

## Architecture

### Socket Layout

| Socket | Direction | Pattern | Port | Message Types | Purpose |
|--------|-----------|---------|------|---------------|---------|
| **State Stream** | sim → client | PUB/SUB | 5556 | `StateUpdate` | Broadcasts simulation state every tick |
| **Synchronous Control** | sim ↔ client | DEALER/DEALER | 5557 | `ControlRequest`/`ControlReply` | Requests control input in sync mode |
| **Admin Commands** | client ↔ sim | REQ/REP | 5558 | `AdminCommand`/`AdminReply` | Remote control (pause, reset, params, etc.) |
| **Asynchronous Control** | client → sim | PUB/SUB | 5559 | `ControlAsync` | Client publishes control commands in async mode |
| **Marker Stream** | client → viz | PUB/SUB | 5560 | `MarkerArray` | Visualization markers from clients |

## Usage

### Enabling Communication

In the GUI:
1. Open the "Communication" section in the Admin panel
2. Toggle "Enable ZMQ" to start the communication server
3. Select "Synchronous" or "Asynchronous" mode (Asynchronous is default)
4. Configure timestep (ms), control period (ms), and control delay (ms) in the Simulation Control panel (changes apply on reset)

### Operating Modes

#### Asynchronous Mode (Default)
- Simulator runs continuously without waiting for client
- Client publishes control commands on port 5559 (PUB/SUB)
- Simulator uses latest control input received from client
- Sample-and-hold: maintains last control until new one arrives
- Non-blocking, suitable for real-time visualization

#### Synchronous Mode
- Simulator queues control requests every *control period* milliseconds (converted to ticks using dt)
- Requested commands are applied after the configured *control delay* (also tick-aligned)
- While waiting for the apply tick, the simulator continues stepping with the previous control
- Once the delay elapses the simulation stalls until a reply arrives (1 second timeout window)
- On timeout: simulation pauses automatically and logs a warning
- Uses DEALER/DEALER pattern for robust timeout handling (no socket resets)
- Deterministic, ideal for step-through debugging in Jupyter

### Message Format

All messages use Protocol Buffers (see `messages.proto`). Key types:

- **StateUpdate**: Contains car position, velocity, yaw, yaw_rate
- **ControlAsync**: Steering angle (rad) and longitudinal acceleration (m/s²) for async mode
- **ControlRequest**: Includes current state and simulation tick (sync mode)
- **ControlReply**: Steering angle (rad) and longitudinal acceleration (m/s²) (sync mode)
- **AdminCommand**: Commands like PAUSE, RUN, STEP, RESET, SET_PARAMS, SET_CONTROL_MODE, SET_SIM_CONFIG, GET_SIM_CONFIG
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
- Manages 4 sockets: state PUB, control DEALER (sync), control SUB (async), admin REP
- State publishing happens every tick (non-blocking)
- Async control: polls SUB socket for latest control
- Sync control: DEALER socket for robust request/reply, 500ms timeout
- Connection state automatically tracked via successful responses
- No socket resets needed (DEALER handles timeouts gracefully)
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


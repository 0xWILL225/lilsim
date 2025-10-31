# lilsim
A simple 2D simulator for Formula Student Driverless.

## Quick Start

### Prerequisites
- CMake 3.20+
- C++20 compiler (clang/gcc/msvc)
- vcpkg dependencies (automatically handled): `glfw3`, `imgui`, `eigen3`, `spdlog`, `protobuf`, `zeromq`, `cppzmq`
- wgpu-native (automatically downloaded on first build)

### Build & Run
```bash
cmake --preset debug
cmake --build --preset build-debug -j
./run_debug.sh
```

### Controls
- **WASD**: Drive the car (W=forward, S=brake, A=steer left, D=steer right)
- **Mouse scroll**: Zoom in/out
- **Close window**: Exit

---

# Driverless Algorithm Prototyping Simulator — Concept Overview

## Purpose

A **simple, modular 2D simulator** for rapid prototyping of algorithms for **state estimation (localization/SLAM), path planning, and control** in Formula Student Driverless.  
The focus is on **determinism**, **debuggability**, and **ease of use** from a **Jupyter notebook**, enabling step-by-step algorithm development and visualization.

---

## Core Modules

### 1. Scene Module

- The **single source of truth** for the simulation state.
- Stores:
    - Car state (dynamic bicycle model)
    - Cone positions
    - Transforms (frame tree)
    - Optional future extensions (other vehicles, sensors, obstacles)
- **Access pattern:**
    - Simulator owns the write access.
    - Other modules (Visualization, external clients) have read-only access.
#### Implementation

- **Double-buffered design:**  
    Two `Scene` buffers: one active (“front”) and one being written (“back”).  
    Simulator writes to back, then atomically swaps pointers.  
    Readers always access the stable “front” snapshot.
    
- **Result:** lock-free access; suitable for 1 kHz simulation vs 60 Hz visualization.

---

### 2. Simulator Module

- Runs at a **fixed timestep (`dt`)** and updates the Scene.
- Responsible for physics integration (dynamic bicycle model, noise models, sensors).
- Owns the Scene during simulation.
- Supports two operating modes:

#### a. Synchronous Mode

- Simulator pauses every `K` ticks (control period) and requests control input from the client:
    - Sends `ControlRequest{tick, deadline, state}`.
    - Waits for `Control{tick, u}` or applies timeout policy (hold/brake/default).
- Deterministic and ideal for Jupyter step-through operation.

#### b. Asynchronous Mode

- Client sends controls freely (`Control{target_tick, u}`).
- Simulator uses the **latest control** whose timestamp ≤ current tick (sample-and-hold).
- No waiting; continuous loop execution.

#### Common

- Every message carries `tick` and `sim_time`.
- Fixed `dt` + seeded RNG → deterministic replay.
- `ControlAck{tick}` messages confirm simulator progress.

---
### 3. Visualization Module

- Renders the current Scene state and user-defined markers.
- Runs at 30–60 FPS, reading the “front” Scene buffer.
- Independent rendering thread.

#### Rendering Stack

- **Dear ImGui + wgpu-native
    - Portable (Metal/D3D12/Vulkan via WebGPU backend)
    - No Rust required
    - Future-proof toward WebGPU
    - Uses `imgui_impl_wgpu` backend (officially supported)
- Uses GLFW for windowing and surface creation.

---
## Marker System

### Option 1: RViz-like (chosen for v1)

- Simple flat registry `(namespace, id)`.
- Primitive types: points, lines, meshes, text, transforms, etc.
- Each marker has color, scale, and optional **TTL (time-to-live)**.
- Visibility toggles per namespace.
- Easily extended later with `parent_id` for lightweight grouping.

### Option 2 (future): Hierarchical / OpenUSD-inspired

- Scene graph with primitives, transforms, and composition.
- Deferred until core simulator is stable.

---
## Transforms and Frames
- SE(2) transforms should generally be stored as 2D homogenous transform matrices `Eigen::Isometry2d`
- Every simulated object holds an SE(2) transform
- Every visualized marker holds an SE(2) transform and a frame id (frame id = name of parent marker or simulated object)
- Visualization module maintains a lightweight **transform tree**:
    - `add_transform(parent, child, T, stamp, static)`
    - `resolve_to_world(frame, t, out T_world_frame)`
- Tree anchored at `world` (or chosen world frame).
- Supports both static and dynamic transforms.
- On marker or sensor message reception:
    - If frame known → immediately resolve to world.
    - If unknown or outdated → reject and log (never silently disappear).
- Optional interpolation/extrapolation for dynamic frames.
---
## Communication Model

### Internal (in-process)
- **Simulator ↔ Scene:** Direct shared memory (pointers + atomics).
- **Simulator ↔ Visualization:** Scene double-buffer read-only access.
### External (to Python/Jupyter)
- **Transport:** **ZeroMQ + Protobuf
    - PUB/SUB for state updates and control requests.
    - PUSH/PULL or REQ/REP for controls and admin commands.
- Lightweight, fast, and notebook-friendly.
- In the future, gRPC may be added for externalized admin or cloud use.
---
## Jupyter Notebook Integration

- Python SDK provides:
    - Connection setup and message handling via `pyzmq`.
    - Callback registration:
        - `on_state(state)`
        - `on_control_request(req)`
    - Helper functions like `step(n)` (blocks until `tick += n`).
        
- Typical workflow:
    1. User defines car and track parameters.
    2. Sends initialization message to simulator.
    3. Registers callbacks for control and perception.
    4. Runs cell to start communication.
    5. Uses visualization window to start/step simulation.
    
- Notebook remains **“locked”** during execution (one running cell).  
    Simplicity and safety preferred over background interactivity.
---
## Key Features

- Deterministic simulation (record/replay ready)
- Step-through debugging and visualization
- Minimal dependencies, modular design
- High-frequency simulation (1 kHz) with decoupled rendering (60 Hz)
- Immediate visual feedback and marker overlays
- Extensible: multiple frames, noise models, or additional sensors

---
## Future Directions

- Integrate real-time logging and replay.
- Add more simulated sensors (IMU, lidar).
- Support driver-in-the-loop experiments (single process shared memory).
- Optional hierarchical scene representation (OpenUSD-inspired).

---
## TL;DR

A **modular, deterministic C++ simulator** built for **rapid Driverless algorithm prototyping**.
- Core: Scene (truth), Simulator (physics/control), Visualization (rendering).
- Fast in-process data sharing, external Python interface via ZeroMQ.
- Step-through control loop and RViz-style visualization, with future-ready WebGPU rendering.

---
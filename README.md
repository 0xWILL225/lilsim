# lilsim
A simple 2D simulator for Formula Student Driverless.

<img src="gifs/lilsim_demo.gif" width="1000" height="562" title="pure-pursuit-demo"> 

## Quick Start

### Prerequisites
- CMake 3.20+
- C++20 compiler (clang/gcc/msvc)
- vcpkg dependencies (automatically handled): `glfw3`, `imgui`, `eigen3`, `spdlog`, `protobuf`, `zeromq`, `cppzmq`, `yaml-cpp`
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
- **Tab**: Change camera mode

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
    - Car state
    - Cone positions
    - Sensors
- **Access pattern:**
    - Simulator module owns the write access.
    - Other modules (Visualization, external clients) have read-only access.
#### Implementation

- **Double-buffered design:**  
    Two `Scene` buffers: one active (“front”) and one being written (“back”).  
    Simulator writes to back, then atomically swaps pointers.  
    Readers always access the stable “front” snapshot.
    
- **Result:** lock-free access; suitable for 1 kHz simulation and 60 Hz visualization.

---

### 2. Simulator Module

- Runs at a **fixed timestep (`dt`)** and updates the Scene.
- Responsible for physics integration (stepping vehicle model, noise models, sensors).
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
### 3. Visualization Module (GUI)

- Renders the current Scene state and user-defined markers.
- Runs at 60 FPS, reading the “front” Scene buffer.
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

### Option 2 (future): Hierarchical / OpenUSD-inspired

- Scene graph with primitives, transforms, and composition.
- Deferred until core simulator is stable.
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
- High-frequency simulation (~1 kHz) with decoupled rendering (60 Hz)
- Immediate visual feedback and marker overlays
- Extensible and flexible: car models, noise models, sensors models

---
## TL;DR

A **modular, deterministic C++ simulator** built for **rapid Driverless algorithm prototyping**.
- Core: Scene (truth), Simulator (physics/control), Visualization (rendering).
- Fast in-process data sharing, external Python interface via ZeroMQ.
- Step-through control loop and RViz-style visualization, with future-ready WebGPU rendering.

---
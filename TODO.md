
-  **(Next phase, after MVP)**
    -  Replace kinematic with dynamic bicycle + simple actuator dynamics
    -  ZeroMQ + Protobuf IPC stub (state PUB, admin REQ/REP, control PUSH)
    -  Python SDK (for Jupyter notebook client) with `step(n)` helper

#### Later on some time in the future
 - **Determinism hooks**
    -  Fixed time base (no wall-clock in physics)
    -  RNG seeded by `(seed ^ tick)` (even if unused yet)
    - Future: Per-tick state checksum (xxhash) for debugging determinism
-  **Logging**
    -  Set it up with the tracy profiler 
    -  Ring buffer of `(tick, control, state)`
    -  ImGui button **Save CSV** (to `logs/` with timestamp filename)
    - `spdlog` init + error console in ImGui
-  **Repo polish**
    -  `clang-format` target; CI build (Linux + Windows at least)
    -  `README` quickstart guide: vcpkg setup, configure, build, run`L`
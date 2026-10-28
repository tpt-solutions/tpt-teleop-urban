# tpt-teleop-urban — Project TODO

Smart City & Autonomous Vehicle Operations middleware. Sister repo to `tpt-teleop`
(video/control transport) and the other `tpt-teleop-*` domain repos
(`tpt-teleop-fleet`, `tpt-t-agri`, `tpt-teleop-marine`, `tpt-teleop-medical`,
`tpt-teleop-defense`, `tpt-teleop-construction`, `tpt-teleop-space`,
`tpt-teleop-inspection`). Repository retains the full `tpt-teleop-urban` name;
all internal crates use the simplified `tpt-t-urban-` prefix. License:
MIT OR Apache-2.0. Copyright TPT Solutions.

Source docs: [spec.txt](spec.txt) (primary design doc), [bridge spec.txt](bridge%20spec.txt)
(tpt-teleop integration contract).

Phases are ordered dependency-first: foundation → cross-repo bridge → core →
V2X comms → perception → prediction → compliance → safety → privacy →
vehicle/delivery/infrastructure control → edge/hardware layer → thread-per-core
integration → teleop bridge → sim → system integration → certification →
hardware bring-up → hardening/release.

---

## Phase 0 — Repository & Workspace Setup
- [x] `git init`, initial commit of spec docs
- [x] Cargo workspace `Cargo.toml` (resolver "3", edition 2024, license
      `MIT OR Apache-2.0`, authors "TPT Solutions"), `crates/*` members,
      shared workspace deps table — mirror sister repos' workspace layout
- [x] `LICENSE-MIT` and `LICENSE-APACHE` at repo root (TPT Solutions, 2026)
- [x] `.gitignore` (`target/`, etc.)
- [x] `deny.toml` (cargo-deny): MIT/BSD/ISC/Zlib allow-chain, explicit
      Apache-2.0-only deny, `[[licenses.clarify]]` overrides for dual-licensed
      deps (`rkyv`, `windows-sys`, `proc-macro2`/`syn`/`quote`, `libc`),
      encodes the §7 dependency matrix (bans heavy ASN.1 compilers, OpenCV/
      SLAM frameworks, `serde`/`bincode`/`prost`, `tokio`/`crossbeam`,
      `openssl`/`rustls`, heavy robotics frameworks like AutoWare, in favor
      of the custom lightweight choices)
- [x] `.github/workflows/ci.yml` — build-test matrix (linux/macos/windows),
      cross-compile check, `RUSTFLAGS: -D warnings`, plus a separate nightly
      job scoped to `tpt-t-urban-predict` and `tpt-t-urban-privacy`
- [x] `.github/workflows/lint.yml` — fmt + clippy (`-D warnings`, stable +
      nightly job for predict/privacy) + cargo-deny action
- [x] `rust-toolchain.toml` — stable default; document the nightly override
      needed for `tpt-t-urban-predict` (SIMD IMM Kalman filter) and
      `tpt-t-urban-privacy` (SIMD blur), both using `portable_simd`
- [x] Root `README.md` (Status / Workspace Crates / Zero-Allocation Data
      Path / Developer Experience / Licensing — mirror sister repos'
      section structure)
- [x] `CHANGELOG.md`, `SECURITY.md`
- [x] `docs/ARCHITECTURE.md` skeleton (workspace map, crate responsibilities,
      thread-per-core layout from spec.txt §4.5)
- [x] `docs/quickstart.md` stub
- [x] Stub empty crate skeletons (Cargo.toml + lib.rs) for all crates below

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (upstream in `tpt-teleop`)
> Status: the crate now exists upstream in `tpt-teleop` at c701ebe, and this
> repo pins it as a git dependency (`Cargo.toml`, `[workspace.dependencies]`).
> Upstream differs from the bridge spec in two places, and the urban side
> follows upstream:
> (1) `ControlCommand` is the flat 56-byte POD from `tpt-t-core::ser`, not the
> spec's `operator_id`/`InputState`/`ButtonState` struct. The operator id is
> given once on engage and held by `DomainSession`.
> (2) The handover states use the spec's names (`HandoverState`), and map onto
> the core `Mode` (`Auto`, `Assist`, `FullTeleop`, `EmergencyStop`).
> Upstream's own tests were not run here. The workspace builds against the crate
> and passes its own tests.
- [x] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/` (upstream)
- [x] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`, plus `on_emergency_stop`
- [x] Define wire types with `rkyv::Archive`: `SensorFeed`, `DomainState`,
      `DomainTelemetry`, `HapticFeedback`, `VideoFrame`, `PointCloud`,
      `AudioStream` (upstream names; `InputState`/`HapticCmd` are not in upstream)
- [x] Reconcile aspirational `ControlCommand` against the flat POD (resolved
      upstream: the POD is the wire type, and the operator id is per session)
- [x] Universal safety state machine: `HandoverState` with `AUTONOMOUS →
      REQUESTING_TELEOP → TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`,
      plus `EMERGENCY_STOP` transitions (`DomainSession` enforces them)
- [x] `request_teleop_assistance()` API surface (`TeleopLink` trait)
- [x] Unit tests + doc examples; git dependency in the urban workspace

## Phase 2 — `tpt-t-urban-core`
- [x] Central deterministic event loop
- [x] Vehicle state machine: Parked → Routing → Driving → Stopped → Emergency
- [x] Lock-free message bus for inter-crate event routing
- [x] rkyv wire-type prelude shared by other urban crates
- [x] Zero-lock hot-path audit passes (`tools/lock-audit.sh`; sister-repo
      script was not on disk, so this one was written fresh)

## Phase 3 — `tpt-t-urban-v2x`
> Status: X.691 UPER primitives are verified against the standard text. The BSM
> core-data decoder is written but its J2735 field ranges are UNVERIFIED (the
> SAE ASN.1 module is paywalled). MAP, SPAT and SRM are not started. The fuzz
> tests and the <10µs benchmark are not done.
>
> Update: a random-input decoder test is in. Host timing for one core-data
> decode is about 0.3 µs against the 10 µs budget. A property test now drives
> every UPER primitive with random bytes and extreme bounds (coverage-guided
> fuzzing is still not done). MAP, SPAT and SRM are still not started. Writing those decoders without the SAE ASN.1 module would mean
> guessing at wire formats, so they stay open.
>
> Update: coverage-guided fuzz targets for the BSM core-data decoder and the UPER
> primitives are in `crates/tpt-t-urban-v2x/fuzz`. Each ran for two minutes on the
> Windows nightly with no crashes, about 7 million executions per target. The
> run needs the MSVC ASan runtime on `PATH`. The `<10µs` benchmark is still open.

- [ ] Zero-copy ASN.1 UPER parser reading raw UDP packets from the C-V2X
      radio directly into pre-allocated rkyv structs
- [ ] BSM (Basic Safety Message) parsing, <10µs per message, zero heap
      allocation
- [ ] MAP, SPAT (Signal Phase and Timing), and SRM (Signal Request Message)
      message parsing
- [ ] Non-Line-of-Sight (NLOS) BSM relay handling (vehicles around blind
      corners/behind trucks)
- [x] Fuzz/property tests for the zero-copy UPER parser against malformed
      frames (cargo-fuzz targets plus a property test; a CI fuzz job is not wired)

## Phase 4 — `tpt-t-urban-perception`
> Status: GNSS kinematic jump detection, LiDAR fallback with hysteresis, and
> curb edge detection from a height profile, and inverse-variance fusion of two
> position estimates are done. A 2D detector groups LiDAR points into objects
> (`perception::detect`), and an alpha-beta tracker keeps their identities and
> velocities across frames (`perception::track`). An integration test runs the
> sim's LiDAR into both: a moving box is tracked at about 2 m/s, and a static box
> at rest. Open: 3D detection, classification, fusion of real sensor streams, and
> any sensor beyond the simulated LiDAR.
- [ ] Multi-sensor fusion pipeline: LiDAR, Radar, Camera, Ultrasonic
- [~] 3D object detection, tracking, and classification (2D detection and
      tracking from LiDAR points are done, with no classification and no 3D)
- [ ] GNSS multipath/anomaly detection (e.g. 5-meter position jump in urban
      canyons)
- [ ] Seamless fallback to LiDAR-based localization on GNSS anomaly,
      maintaining sub-10cm accuracy
- [ ] Curb/sidewalk edge detection support for `tpt-t-urban-delivery`
- [ ] Ultrasonic curb-mounting verification support

## Phase 5 — `tpt-t-urban-predict`
> Status: two-model IMM filter and trajectory cone are done as a scalar
> reference. A `simd` feature adds an eight-wide `portable_simd` kernel for
> batched constant-velocity propagation, tested against the scalar path on
> nightly. Host timing for a 50-agent IMM frame is about 6 µs against the 2 ms
> budget. A batched IMM (`predict::imm_batch`) runs the same two-model filter
> for many agents in structure-of-arrays form. One generic kernel runs one lane
> on stable and eight `f32x8` lanes under `simd`, and the two agree bit for bit.
> It matches the per-agent `Imm` within 1e-3 over a turn. Host timing for 50
> agents: about 5.8 µs on stable, 3.1 µs on nightly, against 7.4 to 8.8 µs for
> the per-agent filter. Open: a crate-local nightly toolchain, and target-hardware
> timing.
- [ ] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only (rest of workspace stays stable)
- [x] Custom SIMD-accelerated Interacting Multiple Model (IMM) Kalman Filter
      (`imm_batch`, eight lanes under `simd`; the lane-width choice is the
      kernel's, not hand-tuned)
- [x] Parallel kinematic-state processing of 100+ dynamic agents via
      `portable-simd` (batched IMM at 128 agents: 16 µs on stable, 6.6 µs with
      `simd`, host timing against the 2 ms budget)
- [ ] Probabilistic trajectory cone generation over a 3-second horizon,
      accounting for human unpredictability (pedestrians, cyclists, vehicles)
- [ ] Benchmark: <2ms per frame at 30Hz for a busy intersection (50+ agents)

## Phase 6 — `tpt-t-urban-compliance`
> Status: stop-sign / speed-limit FSM with reasons, and an Ed25519-signed,
> SHA-256 hash-chained ledger, are done. The ledger is now durable: each append
> is synced before it returns, and a torn final record is cut off on open
> (`compliance::store`). Power-loss behavior is tested with simulated torn
> writes, not a real power cut on target storage. Open: full rule set, and the
> <50µs timing check.

- [ ] Deterministic Finite State Machine encoding local traffic rules (stop
      signs, right-of-way, speed limits)
- [ ] Full intersection-state evaluation in <50µs
- [ ] Cryptographically signed (Ed25519 via `ring`), human-readable
      justification generated for every decision (yield/proceed/stop)
- [x] Hash-chained, append-only immutable audit ledger, surviving sudden
      power loss (verified with simulated torn writes; target storage not yet tested)
- [~] Logging integration: sensor data, V2X messages, planner outputs, and
      FSM decisions all recorded to the black-box ledger (`compliance::journal`
      encodes records. The sim writes guardian interventions and the run outcome
      through `sim::journal`, and the chain verifies after a reopen. Sensor data,
      V2X messages, planner outputs and FSM decisions have no producer to record
      from yet)

## Phase 7 — `tpt-t-urban-safety`
> Status: guardian logic is done (latched AEB from TTC and braking sufficiency,
> latched MRM from planner heartbeat, reset at rest). The sufficiency test now
> requires the planner to stop 1 m short of the obstacle, after a 50 ms reaction.
> The TTC threshold grows with the time a full stop takes. A seeded sweep of
> crossings (`tpt-t-urban-sim`, `sweep` module) found the old rules let a stop
> end inside the pedestrian contact zone, and it now finds no avoidable
> crossing hit on nine seeds of 2000. The shoulder pull-over path and its
> lateral-acceleration bound are in `tpt-t-urban-car::pull_over`. In the sim, a
> guardian MRM drives the vehicle along the pull-over path and it stops on the
> shoulder (`tpt-t-urban-sim::mrm`). `pull_over::choose` picks the pull-over when
> the shoulder fits and the in-place brake when it does not. The sim's closed-loop
> planner (`tpt-t-urban-sim::drive`) follows a route at the speed profile, and
> when its heartbeat stalls the guardian's MRM hands over to that response. The
> planner in the sim is not the production planner, which is not in this repo.
> Open: running on a separate partition, the <50 ms budget (needs measurement on
> target), and dual-channel brake failover.

- [ ] Independent, redundant safety "guardian" state machine (separate
      compute partition from the primary planner)
- [ ] Automatic Emergency Braking (AEB) trigger, <50ms reaction if the
      primary planner fails to react
- [~] Minimal Risk Maneuver (MRM) — e.g. safe pull-to-shoulder — when AEB
      alone is insufficient (a guardian MRM hands a stalled planner over to the
      pull-over, which stops on the shoulder in the sim, or to braking in place
      when no shoulder fits; the planner is the sim's, not production)
- [ ] Deceleration-sufficiency validation against `tpt-t-urban-car`'s
      planned trajectory
- [ ] Dual-channel steer-by-wire/brake-by-wire failover monitoring and
      automatic channel switch on fault

## Phase 8 — `tpt-t-urban-privacy`
> Status: in-place region blur, a redaction gate (only a redacted frame reaches
> a sink), and the visualizer comparison hook are done. Open: the face and plate
> detection model, the <3 ms measurement, and the SIMD kernel with its nightly
> toolchain.
- [ ] Crate-local nightly toolchain override + `#![feature(portable_simd)]`
      (shares the nightly CI job scope with `tpt-t-urban-predict`)
- [ ] Lightweight SIMD-optimized face/license-plate detection model
- [ ] In-place Gaussian blur mask applied directly on the pre-allocated
      zero-copy ring-buffer frame, before the frame reaches recording/
      teleoperation
- [ ] Benchmark: <3ms added latency
- [ ] Guarantee: no raw PII (unblurred face/plate) ever written to disk or
      transmitted
- [ ] Side-by-side raw-vs-anonymized verification hook for the visualizer

## Phase 9 — `tpt-t-urban-car`
> Status: speed profile (curvature, accel and brake limits), pure pursuit
> tracking, and the GLOSA advisory are done. Fastest-route planning over a
> directed road graph (`routing` module) replans around closed edges. Open:
> feeding closures from live V2X and map data, and emergency-vehicle SRM yield.
- [ ] Autonomous vehicle trajectory planning and decision-making for 4-wheel
      vehicles (robotaxis and autonomous transit shuttles)
- [ ] Path generation, speed profiling, smooth kinematic control
- [ ] GLOSA (Green Light Optimal Speed Advisory): consume `tpt-t-urban-v2x`
      SPAT messages to smoothly adjust speed and arrive at a stop line
      exactly as a light turns red, avoiding harsh stops
- [ ] Dynamic re-routing around road closures, construction zones, or heavy
      congestion using real-time V2X and map data
- [ ] Emergency Vehicle Preemption: yield/pull-over response to an
      ambulance/fire-truck SRM broadcast

## Phase 10 — `tpt-t-urban-delivery`
> Status: curb mount with ultrasonic verification, pedestrian yield with
> hold-off, and the compartment lock with a constant-time code check are done.
> A static-obstacle detour planner for a straight sidewalk is in
> `delivery::sidewalk`. It returns the smallest detour that clears every obstacle
> with the bot's footprint, and stop when none fits. Open: navigation off a
> straight sidewalk, moving obstacles, sensor-driven obstacle input, and the
> jaywalker integration (Phase 16).
- [ ] Curb detection and precise curb-mounting (verified by ultrasonic
      sensors from `tpt-t-urban-perception`)
- [~] Sidewalk navigation and obstacle avoidance (static obstacles on a straight
      sidewalk: detour or stop; no route following, no moving obstacles)
- [ ] Pedestrian right-of-way yielding with audible/visual polite signaling
- [ ] Secure package compartment lock/unlock management
- [ ] Jaywalker-reaction integration: consume `tpt-t-urban-predict` trajectory
      cones and command smooth deceleration ahead of the guardian's AEB
      backstop

## Phase 11 — `tpt-t-urban-traffic` (infrastructure-side deployment)
> Status: fixed-time signal controller with yellow and all-red clearance, and
> SRM preemption, are done. Congestion optimization has its functions: a
> demand-proportional green split with min and max bounds, corridor offsets for a
> green wave, and a controller hook that applies new greens at the next phase
> start. Open: a demand source for the optimizer (queue counts from V2X or
> detection), and SPAT/MAP broadcast.
- [ ] Intersection management (signal phasing coordination)
- [~] Congestion optimization across managed intersections (functions and
      controller hook done; no demand source feeds them yet)
- [ ] Priority routing for emergency vehicles or transit (SRM issuance)
- [ ] V2X broadcast integration (SPAT/MAP outward to approaching vehicles)

## Phase 12 — `tpt-t-urban-edge` (`no_std`, bare-metal/RTOS)
> Status: `no_std` actuation limiter, watchdog, and dual-channel failover are
> done. The crate builds for `thumbv6m-none-eabi`, `thumbv7em-none-eabihf`,
> `thumbv8m.main-none-eabihf` and `riscv32imac-unknown-none-elf`. Builds only, not
> run on hardware. Open: bare-metal drivers, and CAN/Ethernet actuation.
- [ ] Bare-metal/RTOS execution layer for vehicle actuation
- [ ] Direct, deterministic steer-by-wire control
- [ ] Direct, deterministic brake-by-wire control
- [ ] Direct, deterministic throttle control
- [ ] Hardware watchdog integration + watchdog-feeding logic
- [x] `no_std` build validated for target embedded platforms (builds for four
      targets; hardware runs are open under Phase 18)

## Phase 13 — Thread-Per-Core Integration (spec.txt §4.5)
> Status: a schedulability check for the spec's pinned layout (per-core
> utilization and chain latency) is in `tpt-t-urban-core::schedule`. Thread
> pinning is in `tpt-t-urban-core::affinity`: a thread can be pinned to a core
> on Windows (tested) and Linux (compile-checked, not run). Open: running the
> stages on pinned threads in a single program, measured worst cases, and the
> target-hardware latency sign-off.
- [ ] Core 0: io_uring ingest of raw LiDAR/Camera/Radar data into
      slab-allocated ring buffers
- [ ] Core 1: `tpt-t-urban-perception` + `tpt-t-urban-predict` pinned,
      running at 30Hz
- [ ] Core 2: `tpt-t-urban-car` planning + `tpt-t-urban-compliance` FSM
      pinned, running at 30Hz
- [ ] Core 3: `tpt-t-urban-edge` actuation + `tpt-t-urban-safety` guardian
      pinned, running at 100Hz
- [ ] Zero-context-switch / zero-lock-contention verification across the
      pinned-core layout
- [ ] End-to-end deterministic max-latency measurement and budget sign-off

## Phase 14 — `tpt-t-urban-teleop` (adapter crate)
> Status: `TeleopAdapter` implements upstream `DomainTeleopInterface`
> (`tpt-t-domain-bridge`, pinned at c701ebe with `tpt-t-core` on the same rev).
> It uses the upstream `ControlCommand` (56-byte POD) and `StateMachine`. Each
> command passes the wire, mode, traffic-law and actuator gates. Engage and
> disengage are tracked per operator, and an emergency stop latches. Tests drive
> the adapter through `DomainSession` (`tests/session.rs`).
> Handover trigger: a latched guardian MRM (stale planner heartbeat) requests
> assistance as `Degraded`, through `assistance_reason`. AEB does not, because
> it clears on its own. Return to autonomy waits until the MRM is cleared at
> rest (`ready_to_return`, `clear_mrm`). Tests are in `tests/handover.rs`.
> An operator may engage while an MRM or AEB is latched. The guardian's override
> then applies to the operator: throttle is removed and the brake is full
> (`TeleopAdapter::set_guardian`). The operator can steer and stop, and cannot
> drive the vehicle until the guardian clears.
> `Assist` blending is not implemented (steering passes, throttle held). The
> bridge spec defers continuous blended control, so this is not a spec gap.
> `get_domain_state` takes its position from `update_localization`, which clears
> the sensor fault. `set_fault` raises or clears fault bits. Battery has no source.
> `get_sensor_feed` carries video and point clouds. Video is blurred by
> `push_video` before it is queued, so only the redacted copy leaves the vehicle.
> Point clouds are passed through. Haptics and audio have no source.
> Traffic law: `update_compliance` runs `compliance::evaluate`, advances the
> stop-sign approach state, and caps the actuators with its decision.
> `begin_stop_approach` must be called when a route enters a stop sign, since the
> FSM ignores stop signs until then.
> Emergency stop applies full brake and zero throttle on the first tick. It
> bypasses the slew limit (`Limiter::emergency_brake`), and steering holds.
> Trade-off accepted: a stop may jerk the vehicle once, where a slew-limited
> stop would take four ticks.
- [x] Depend on `tpt-t-domain-bridge` (Phase 1) and implement
      `DomainTeleopInterface`
- [x] Implement `on_teleop_engage`/`on_teleop_disengage` handover triggered
      by `tpt-t-urban-safety`. The trigger covers a latched MRM only. The spec's
      "construction zone the planner cannot navigate" case needs a planner
      signal that does not exist yet
- [x] Implement `on_control_command`: steering-wheel angle / pedal position
      (or joystick) → steer-by-wire and brake-by-wire commands via
      `tpt-t-urban-edge`. The actuation is stored in `last_actuation()`
- [~] Implement `get_domain_state` (vehicle position, battery/fault status)
      and `get_sensor_feed` (video, LiDAR, telemetry, haptic). Position, faults,
      video and point clouds are wired. Battery, haptics and audio have no source
- [x] Traffic-law enforcement during teleoperation: apply
      `tpt-t-urban-compliance` FSM constraints (speed limits, stop signs)
      even while a human is driving (`update_compliance` feeds the gate)
- [x] Privacy protection during teleoperation: route video feeds through
      `tpt-t-urban-privacy` before transmission to the remote operator
      (`push_video` redacts before queueing)
- [x] Return-to-autonomy handling once the vehicle clears the handover
      condition. Gated on the guardian's MRM latch being cleared at rest. The
      construction-zone-cleared case waits on the same planner signal as above
- [x] Reconcile aspirational `ControlCommand` against upstream's POD type
      (resolved upstream: the POD is the wire type, the operator id is per session)

## Phase 15 — `tpt-t-urban-sim` + Visualizer
> Status: a kinematic bicycle plant and a crossing-pedestrian scenario (noisy
> measurements → IMM → guardian → plant) are done. A seeded sweep over crossings
> (`sweep` module, `cargo run --release -p tpt-t-urban-sim --example sweep`) checks
> the guardian on every crossing that full braking could have avoided. A GNSS model
> with multipath and denial windows drives the localization source selector, and
> the chain is tested through it (`tests/gnss_canyon.rs`). A 2D LiDAR casts rays
> against boxes (`sim::lidar`). Neighbour BSMs are encoded, sent over a lossy
> delayed channel, and decoded (`sim::v2x`). An egui visualizer draws the crossing
> (`visualizer` binary, `viz` feature; it builds and lints, and has not been run
> in a window). A closed-loop planner drives a route, and a stalled planner hands
> over to the MRM (`sim::drive`). Open: a 3D city, camera and radar models, and
> SPAT traffic (no encoder for it yet).
- [ ] Headless simulator: high-fidelity 3D city environments with dynamic
      traffic, pedestrians, and cyclists driven by realistic behavioral
      models (2D boxes and a straight road only)
- [~] Sensor simulation: LiDAR raycasting (2D, boxes; done), camera rendering
      with motion blur, radar point generation (not started)
- [~] V2X network simulation: inject realistic BSM/SPAT traffic with
      configurable latency and packet loss (BSM only, with latency and loss;
      no SPAT encoder)
- [x] GNSS multipath and denial modeling for urban canyon scenarios (noise,
      5 m multipath jumps and denial windows, tested through the source selector;
      the LiDAR side is modeled as ground truth, so sub-10 cm is not shown here)
- [~] Scenario injection: automatically run thousands of edge cases (e.g.
      "pedestrian runs into street at 30mph") to validate the safety
      guardian and compliance FSM (the guardian is swept over 2000 crossings
      per seed, and the compliance FSM is not in the sweep yet)
- [~] `egui` (MIT) visualizer binary: top-down view of the vehicle, the
      pedestrian and the guardian state (2D, one scenario; no predicted
      trajectories or traffic-rule FSM state yet; not run in a window)
- [ ] Visualizer: real-time sensor fusion output (bounding boxes, point
      clouds)
- [ ] Visualizer: privacy blur verification (side-by-side raw vs.
      anonymized frames)
- [ ] Visualizer: V2X message reception and range visualization

## Phase 16 — System Integration & End-to-End
> Status: a last-mile data-flow test runs GLOSA, stop-sign compliance, pedestrian
> yield, curb mount and unlock across the crates, with every decision in a
> verified ledger (`crates/tpt-t-urban-sim/tests/last_mile.rs`). A sensor-driven
> crossing test is also in. Open: the flow driven entirely by sensor models,
> and the cross-crate allocation audit.
- [ ] Full "Last-Mile Urban Delivery" data-flow test (spec.txt §6):
      departure → SPAT-driven smooth speed adjustment approaching an
      intersection → GNSS urban-canyon anomaly with seamless LiDAR fallback
      → jaywalker detected/predicted (1.5s intersection horizon) → smooth
      deceleration commanded + guardian validation (AEB backstop) → face
      blur applied before local SSD write → curb-mount + secure-compartment
      delivery → hash-chained audit log proving detection, correct
      yielding, and safe distance
- [x] Dependency-matrix conformance audit against §7 (confirm no banned
      dependency — heavy ASN.1 compilers, OpenCV/SLAM frameworks, black-box
      NN traffic rules, heavy ML inference, serde/bincode/prost, tokio/
      crossbeam, AutoWare-style frameworks, openssl/rustls — entered the
      tree)
- [~] Cross-workspace zero-allocation/zero-lock verification across all
      urban crates. Zero-lock: `tools/lock-audit.sh` covers core, safety,
      compliance, edge and teleop, and passes. Zero-allocation: each hot-path
      crate has a `tests/allocation.rs` with a counting global allocator and
      a control case. The success paths make zero allocations in the bus,
      guardian, compliance `evaluate`, edge limiter/watchdog/failover, perception
      steps (jump check, source selection, fusion, curb detection), predict
      (IMM step and estimate, scalar propagation), and the teleop hooks.
      The teleop hooks' error paths allocate nothing either, since upstream
      `DomainError::Adapter` now holds a `&'static str` (upstream commit
      198c3fa on `fix/domain-error-static-str`, breaking change, pushed
      manually). Also covered: `EventLoop::step`.
      The predict SIMD kernel is covered too, on nightly with `--all-features`.
      Open: the ledger append, which is audit rather than hot path, so it is not
      checked.
- [x] Full `cargo-deny` MIT-chain audit pass across the workspace (`cargo deny
      check`: advisories, bans, licenses and sources all pass. Duplicate-version
      warnings remain, from `windows-sys` 0.52 and 0.60, and are non-blocking)

## Phase 17 — Regulatory & Certification Readiness
> Status: a working FMEA for the AEB and MRM path is in
> `docs/safety/aeb-fmea.md`, with each mitigation marked by whether it exists in
> code. The ISO 26262, ISO 21448 and UL 4600 reviews themselves are not done and
> need the people and the hardware.
- [ ] ISO 26262 (Functional Safety, ASIL-D concepts) compliance review
- [ ] ISO 21448 (SOTIF — Safety of the Intended Functionality) compliance
      review
- [ ] UL 4600 compliance review
- [ ] Deterministic-execution and zero-undefined-behavior audit across all
      hot-path crates (core, safety, compliance, edge)
- [ ] Complete auditability review: every FSM decision traceable to a
      signed, human-readable justification in the immutable ledger
- [ ] GDPR/CCPA-by-design audit of the `tpt-t-urban-privacy` pipeline (no
      unblurred PII path exists end-to-end)
- [ ] FMEA (Failure Modes and Effects Analysis) pass on the guardian
      AEB/MRM trigger path

## Phase 18 — Hardware Bring-Up & Field Validation
- [ ] Zero-alloc CAN/Ethernet actuation parser/driver bring-up
- [ ] Hardware-in-the-loop testing harness
- [ ] Physical steer-by-wire/brake-by-wire/throttle driver bring-up on a
      target vehicle (robotaxi or sidewalk delivery bot)
- [ ] Real LiDAR/radar/camera/ultrasonic integration replacing sim
      synthetic data
- [ ] Real C-V2X/DSRC radio integration replacing simulated BSM/SPAT/SRM
      traffic
- [ ] Target embedded platform performance validation against every spec'd
      latency budget on real silicon
- [ ] Geofenced ODD (Operational Design Domain) field trial: full sidewalk
      delivery route or robotaxi loop, start to finish

## Phase 19 — Hardening, Docs & v1.0.0 Release
> Status: workspace clippy, nightly clippy for predict and privacy, a
> random-input decoder test, and a host-machine budget check are done
> (`cargo run --release -p tpt-t-urban-sim --example budgets`). All host
> figures are under budget. Target-hardware measurement is open, as are the
> release steps. Coverage-guided fuzzing now runs for the V2X decoders (Phase 3).
> Crate-level rustdoc is audited: all 13 crates have module docs, and `cargo doc`
> builds clean with the default lints. Item-level docs are complete:
> `RUSTDOCFLAGS="-D missing_docs" cargo doc --workspace --no-deps` passes. The
> 185 undocumented items found earlier are now documented.
- [x] `cargo clippy --workspace --all-targets -- -D warnings` clean
      (stable workspace)
- [x] Nightly clippy pass clean for `tpt-t-urban-predict` and
      `tpt-t-urban-privacy`
- [ ] Performance benchmarks validated against every spec target: <10µs BSM
      parse, <50µs compliance FSM eval, <2ms/frame predict, <3ms privacy
      blur, <50ms guardian AEB reaction, sub-10cm GNSS-denied localization
- [x] Fuzz/property testing for custom zero-allocation parsers (V2X UPER)
      (cargo-fuzz targets for BSM and the UPER primitives, plus property tests;
      a longer campaign and a CI job are still to do)
- [x] Developer docs complete (`docs/quickstart.md`, `docs/ARCHITECTURE.md`,
      crate-level rustdoc). Both docs are written and their commands were run.
      Crate-level rustdoc is audited, and every public item is documented
      (`-D missing_docs` passes).
- [~] README finalized (status section and test instructions updated; wording
      needs a review before the release)
- [ ] Tag `v1.0.0` — "v1 Monolithic Release"

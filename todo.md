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

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (lives in `tpt-teleop` repo)
- [ ] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/`
- [ ] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`
- [ ] Define wire types with `rkyv::Archive`: `ControlCommand`, `InputState`,
      `SensorFeed`, `DomainState`, `DomainTelemetry`, `HapticCmd`/`HapticFeedback`
- [ ] Reconcile aspirational `ControlCommand` (operator_id/InputState/
      ButtonState) against any existing flat POD `ControlCommand` type in
      `tpt-teleop-core/src/ser/cmd.rs` (bridge spec.txt flags this as an
      intentional, unreconciled discrepancy — resolve or document the mapping)
- [ ] Universal safety state machine: `AUTONOMOUS → REQUESTING_TELEOP →
      TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`, plus
      `EMERGENCY_STOP` transitions
- [ ] `request_teleop_assistance()` API surface
- [ ] Unit tests + doc examples; publish path so `tpt-t-urban-teleop` can
      depend on it (path dep during co-development)

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
> decode is about 0.3 µs against the 10 µs budget. MAP, SPAT and SRM are still
> not started. Writing those decoders without the SAE ASN.1 module would mean
> guessing at wire formats, so they stay open.

- [ ] Zero-copy ASN.1 UPER parser reading raw UDP packets from the C-V2X
      radio directly into pre-allocated rkyv structs
- [ ] BSM (Basic Safety Message) parsing, <10µs per message, zero heap
      allocation
- [ ] MAP, SPAT (Signal Phase and Timing), and SRM (Signal Request Message)
      message parsing
- [ ] Non-Line-of-Sight (NLOS) BSM relay handling (vehicles around blind
      corners/behind trucks)
- [ ] Fuzz/property tests for the zero-copy UPER parser against malformed
      frames

## Phase 4 — `tpt-t-urban-perception`
> Status: GNSS kinematic jump detection, LiDAR fallback with hysteresis, and
> curb edge detection from a height profile, and inverse-variance fusion of two
> position estimates are done. Open: fusion of real sensor streams, and 3D
> detection and tracking, which need sensor data and a detector.
- [ ] Multi-sensor fusion pipeline: LiDAR, Radar, Camera, Ultrasonic
- [ ] 3D object detection, tracking, and classification
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
> budget. Open: a SIMD IMM filter itself, a crate-local nightly toolchain, and
> target-hardware timing.
- [ ] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only (rest of workspace stays stable)
- [ ] Custom SIMD-accelerated Interacting Multiple Model (IMM) Kalman Filter
- [ ] Parallel kinematic-state processing of 100+ dynamic agents via
      `portable-simd`
- [ ] Probabilistic trajectory cone generation over a 3-second horizon,
      accounting for human unpredictability (pedestrians, cyclists, vehicles)
- [ ] Benchmark: <2ms per frame at 30Hz for a busy intersection (50+ agents)

## Phase 6 — `tpt-t-urban-compliance`
> Status: stop-sign / speed-limit FSM with reasons, and an Ed25519-signed,
> SHA-256 hash-chained in-memory ledger, are done. Open: full rule set,
> durable power-loss-safe storage, and the <50µs timing check.

- [ ] Deterministic Finite State Machine encoding local traffic rules (stop
      signs, right-of-way, speed limits)
- [ ] Full intersection-state evaluation in <50µs
- [ ] Cryptographically signed (Ed25519 via `ring`), human-readable
      justification generated for every decision (yield/proceed/stop)
- [ ] Hash-chained, append-only immutable audit ledger, surviving sudden
      power loss
- [ ] Logging integration: sensor data, V2X messages, planner outputs, and
      FSM decisions all recorded to the black-box ledger

## Phase 7 — `tpt-t-urban-safety`
> Status: guardian logic is done (latched AEB from TTC and braking sufficiency,
> latched MRM from planner heartbeat, reset at rest). Open: running on a separate
> partition, the <50 ms budget (needs measurement on target), shoulder-pull
> MRM path planning, and dual-channel brake failover.

- [ ] Independent, redundant safety "guardian" state machine (separate
      compute partition from the primary planner)
- [ ] Automatic Emergency Braking (AEB) trigger, <50ms reaction if the
      primary planner fails to react
- [ ] Minimal Risk Maneuver (MRM) — e.g. safe pull-to-shoulder — when AEB
      alone is insufficient
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
> tracking, and the GLOSA advisory are done. Open: re-routing over a road
> graph, and emergency-vehicle SRM yield.
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
> Open: sidewalk navigation and obstacle avoidance, and the jaywalker
> integration (Phase 16).
- [ ] Curb detection and precise curb-mounting (verified by ultrasonic
      sensors from `tpt-t-urban-perception`)
- [ ] Sidewalk navigation and obstacle avoidance
- [ ] Pedestrian right-of-way yielding with audible/visual polite signaling
- [ ] Secure package compartment lock/unlock management
- [ ] Jaywalker-reaction integration: consume `tpt-t-urban-predict` trajectory
      cones and command smooth deceleration ahead of the guardian's AEB
      backstop

## Phase 11 — `tpt-t-urban-traffic` (infrastructure-side deployment)
> Status: fixed-time signal controller with yellow and all-red clearance, and
> SRM preemption, are done. Open: congestion optimization and SPAT/MAP broadcast.
- [ ] Intersection management (signal phasing coordination)
- [ ] Congestion optimization across managed intersections
- [ ] Priority routing for emergency vehicles or transit (SRM issuance)
- [ ] V2X broadcast integration (SPAT/MAP outward to approaching vehicles)

## Phase 12 — `tpt-t-urban-edge` (`no_std`, bare-metal/RTOS)
> Status: `no_std` actuation limiter, watchdog, and dual-channel failover are
> done, and the crate builds for `thumbv7em-none-eabihf`. Open: bare-metal
> drivers, CAN/Ethernet actuation, and validation on other target platforms.
- [ ] Bare-metal/RTOS execution layer for vehicle actuation
- [ ] Direct, deterministic steer-by-wire control
- [ ] Direct, deterministic brake-by-wire control
- [ ] Direct, deterministic throttle control
- [ ] Hardware watchdog integration + watchdog-feeding logic
- [ ] `no_std` build validated for target embedded platforms

## Phase 13 — Thread-Per-Core Integration (spec.txt §4.5)
> Status: a schedulability check for the spec's pinned layout (per-core
> utilization and chain latency) is in `tpt-t-urban-core::schedule`. Open: thread
> pinning, measured worst cases, and the target-hardware latency sign-off.
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
> Status: the teleop safety state machine and the gated command adapter (mode,
> traffic law, actuator limits, privacy redaction) are done. Open: implementing
> `DomainTeleopInterface`, which is blocked on `tpt-t-domain-bridge` in the
> `tpt-teleop` repository.
- [ ] Depend on `tpt-t-domain-bridge` (Phase 1) and implement
      `DomainTeleopInterface`
- [ ] Implement `on_teleop_engage`/`on_teleop_disengage` handover triggered
      by `tpt-t-urban-safety` (e.g. complex construction zone the planner
      cannot navigate)
- [ ] Implement `on_control_command`: steering-wheel angle / pedal position
      (or joystick) → steer-by-wire and brake-by-wire commands via
      `tpt-t-urban-edge`
- [ ] Implement `get_domain_state` (vehicle position, battery/fault status)
      and `get_sensor_feed` (video, LiDAR, telemetry, haptic)
- [ ] Traffic-law enforcement during teleoperation: apply
      `tpt-t-urban-compliance` FSM constraints (speed limits, stop signs)
      even while a human is driving
- [ ] Privacy protection during teleoperation: route video feeds through
      `tpt-t-urban-privacy` before transmission to the remote operator
- [ ] Return-to-autonomy handling once the vehicle clears the handover
      condition (construction zone cleared, etc.)
- [ ] Reconcile aspirational `ControlCommand` (operator_id/InputState/
      ButtonState) against upstream's actual POD type (open decision,
      tracked upstream — same open item as sister repos)

## Phase 15 — `tpt-t-urban-sim` + Visualizer
> Status: a kinematic bicycle plant and a crossing-pedestrian scenario (noisy
> measurements → IMM → guardian → plant) are done. Open: 3D city, sensor
> raycasting, V2X traffic injection, and the egui visualizer. Headless SVG
> rendering of the scenario trace stands in for the visualizer for now
> (`cargo run -p tpt-t-urban-sim --example render_crossing`).
- [ ] Headless simulator: high-fidelity 3D city environments with dynamic
      traffic, pedestrians, and cyclists driven by realistic behavioral
      models
- [ ] Sensor simulation: LiDAR raycasting, camera rendering with motion
      blur, radar point generation
- [ ] V2X network simulation: inject realistic BSM/SPAT traffic with
      configurable latency and packet loss
- [ ] GNSS multipath and denial modeling for urban canyon scenarios
- [ ] Scenario injection: automatically run thousands of edge cases (e.g.
      "pedestrian runs into street at 30mph") to validate the safety
      guardian and compliance FSM
- [ ] `egui` (MIT) visualizer binary: top-down 2D/3D view of the vehicle,
      predicted trajectories of all agents, and the active traffic-rule
      FSM state
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
- [ ] Dependency-matrix conformance audit against §7 (confirm no banned
      dependency — heavy ASN.1 compilers, OpenCV/SLAM frameworks, black-box
      NN traffic rules, heavy ML inference, serde/bincode/prost, tokio/
      crossbeam, AutoWare-style frameworks, openssl/rustls — entered the
      tree)
- [ ] Cross-workspace zero-allocation/zero-lock verification across all
      urban crates
- [ ] Full `cargo-deny` MIT-chain audit pass across the workspace

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
> figures are under budget. Target-hardware measurement is open, as are
> coverage-guided fuzzing and the docs and release steps.
- [x] `cargo clippy --workspace --all-targets -- -D warnings` clean
      (stable workspace)
- [x] Nightly clippy pass clean for `tpt-t-urban-predict` and
      `tpt-t-urban-privacy`
- [ ] Performance benchmarks validated against every spec target: <10µs BSM
      parse, <50µs compliance FSM eval, <2ms/frame predict, <3ms privacy
      blur, <50ms guardian AEB reaction, sub-10cm GNSS-denied localization
- [ ] Fuzz/property testing for custom zero-allocation parsers (V2X UPER)
- [ ] Developer docs complete (`docs/quickstart.md`, `docs/ARCHITECTURE.md`,
      crate-level rustdoc)
- [ ] README finalized
- [ ] Tag `v1.0.0` — "v1 Monolithic Release"

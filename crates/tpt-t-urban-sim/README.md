# tpt-t-urban-sim

Headless urban simulator, scenario injection, and egui visualizer.

## Modules

- `plant`: kinematic bicycle model of the vehicle.
- `scenario`: a pedestrian crossing scenario that runs the whole chain: noisy
  measurements, then IMM prediction, then guardian, then vehicle.
- `render`: visualizer support.

Not implemented yet: 3D city rendering, sensor raycasting, V2X traffic
injection, and the egui visualizer. Those are Phase 15 items in
[todo.md](../../todo.md).

## Testing

```sh
cargo test -p tpt-t-urban-sim
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

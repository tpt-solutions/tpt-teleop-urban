# tpt-t-urban-car

Trajectory planning, speed profiling, and kinematic control for 4-wheel vehicles.

## Modules

- `profile`: path curvature and the speed profile along it.
- `control`: pure pursuit path tracking.
- `glosa`: green-light speed advisory from SPAT timing.
- `routing`: fastest route over a road graph, replanned on closures.

Not implemented yet: emergency-vehicle preemption, which needs SRM input.

## Testing

```sh
cargo test -p tpt-t-urban-car
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

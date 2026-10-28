# tpt-t-urban-perception

Multi-sensor fusion, 3D object detection, tracking, and GNSS anomaly handling.

## Modules

- `gnss`: kinematic GNSS jump detection and LiDAR fallback with hysteresis.
- `curb`: curb edge detection from a lateral height profile.
- `fusion`: inverse-variance fusion of two position estimates.

Not yet implemented: 3D object detection and tracking. Those need sensor data
and a detector model, which this repository does not have.

## Testing

```sh
cargo test -p tpt-t-urban-perception
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

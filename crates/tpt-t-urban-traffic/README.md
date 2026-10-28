# tpt-t-urban-traffic

Infrastructure-side intersection management, congestion optimization, and SRM priority routing.

## Modules

- `controller`: fixed-time signal controller with yellow and all-red
  clearance, and emergency-vehicle preemption driven by SRM.

Not implemented yet: congestion optimization across intersections, and
SPAT/MAP broadcast. Those need the V2X message encoders in
`tpt-t-urban-v2x`.

## Testing

```sh
cargo test -p tpt-t-urban-traffic
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

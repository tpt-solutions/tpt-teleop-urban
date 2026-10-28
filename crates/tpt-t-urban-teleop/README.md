# tpt-t-urban-teleop

Adapter implementing tpt-teleop's `DomainTeleopInterface` for urban vehicles.

## Modules

- `adapter`: turns the upstream `ControlCommand` into actuator commands, gated
  by the upstream mode machine, traffic law, and actuator limits. It also
  redacts video before it leaves the vehicle.
- `handover`: assistance reason, MRM clearance, and return-to-control checks.

`TeleopAdapter` implements `DomainTeleopInterface` from the upstream
`tpt-t-domain-bridge` crate. Wrap it in a `DomainSession` to get the handover
lifecycle. The integration contract is in [bridge spec.txt](../../bridge%20spec.txt).

## Testing

```sh
cargo test -p tpt-t-urban-teleop
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

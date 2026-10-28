# tpt-t-urban-v2x

Zero-copy C-V2X / DSRC parsing for BSM, MAP, SPAT, and SRM messages.

## Modules

- `uper`: X.691 unaligned PER primitives. Verified against the standard.
- `bsm`: BSM Part I core data. The field ranges are NOT verified against
  SAE J2735; see the module docs.

MAP, SPAT and SRM are not implemented yet (Phase 3 in [todo.md](../../todo.md)).

## Testing

```sh
cargo test -p tpt-t-urban-v2x
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

# tpt-t-urban-compliance

Deterministic traffic-law FSM with signed justifications and a hash-chained audit ledger.

## Modules

- `decision`: stop-sign and speed-limit FSM. Each decision carries a reason.
- `ledger`: append-only ledger. Entries are hash-chained and Ed25519-signed.

Implements Phase 6 of [todo.md](../../todo.md). Durable storage and the full
rule set are not implemented yet.

## Testing

```sh
cargo test -p tpt-t-urban-compliance
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

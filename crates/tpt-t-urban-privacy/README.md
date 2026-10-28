# tpt-t-urban-privacy

On-the-fly face and license-plate blurring in the zero-copy frame buffer.

## Modules

- `frame`: grayscale frame buffer and the in-place region blur.
- `redact`: the redaction gate. Only a `RedactedFrame` can reach a `Sink`.

Detection is not implemented here. The face and plate detector is a trained
model and is not in this repository. The redaction path takes detected
rectangles as input, so a detector can be plugged in later.

## Testing

```sh
cargo test -p tpt-t-urban-privacy
```

Part of [tpt-teleop-urban](../../README.md). Licensed under MIT OR Apache-2.0.

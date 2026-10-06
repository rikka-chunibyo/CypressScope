# Feature Modules

Each user-facing device mode belongs in its own subdirectory and exposes a
small `FeatureModule` interface:

- `init` starts hardware and emits startup diagnostics.
- `service` performs one cooperative unit of work.
- private implementation details stay inside the feature directory.

The active mode is selected at runtime through the USB command path and
resolved by `feature_registry.c`. Send `MODE USB`, `MODE LIVE`, or `MODE SCOPE`
followed by CR/LF. The current modes are USB, triggered oscilloscope, and live
feed. The live feed reuses the oscilloscope acquisition implementation because
both modes use the same ADC/DMA hardware. Adding the logic analyzer should add
a `logic_analyzer` directory and register its module without expanding
`main.c`.

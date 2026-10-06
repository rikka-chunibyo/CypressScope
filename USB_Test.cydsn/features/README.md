# Feature Modules

Each user-facing device mode belongs in its own subdirectory and exposes a
small `FeatureModule` interface:

- `init` starts hardware and emits startup diagnostics.
- `service` performs one cooperative unit of work.
- private implementation details stay inside the feature directory.

The active mode is selected in `feature_config.h` and resolved by
`feature_registry.c`. Adding the logic analyzer should therefore add a
`logic_analyzer` directory and register its module without expanding
`main.c`.

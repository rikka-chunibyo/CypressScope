# Logic Analyzer Feature

This feature is reserved for the digital acquisition mode. It should expose a
`FeatureModule` from `logic_analyzer.h` and keep digital sampling, bit packing,
trigger configuration, and protocol metadata inside this directory.

Register it in `feature_registry.c` after its hardware path and output format
are implemented.

# Live Feed Feature

The first live-feed implementation is exposed by the oscilloscope acquisition
module as `Oscilloscope_GetLiveModule()`. It reuses the existing ADC, timer,
DMA ring, and trigger-gate hardware without arming a trigger. This directory
documents the user-facing mode; the implementation stays with the shared
oscilloscope acquisition code until the sampling engine is split out.

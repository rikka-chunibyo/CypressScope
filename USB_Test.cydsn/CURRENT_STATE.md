# CypressScope USB_Test Current State

Last verified: 2026-10-06

## Active Project

- Active firmware project: `USB_Test.cydsn`.
- The project is currently a two-channel hardware-gated capture prototype.
- The intended product will support both oscilloscope capture and a digital logic analyzer mode.
- `DebugUART` is diagnostic-only. `USBUART` is the application communication path.
- `Proper_Capture_Test.cydsn` contains the earlier capture implementation and protocol notes; `USB_Test.cydsn` is the active USB communication project.

## Feature Architecture

- `main.c` is now only the composition root: it enables interrupts, initializes communication, selects the active feature, and runs the common service loop.
- `features/feature.h` defines the small class-like `FeatureModule` contract: `name`, `init`, and cooperative `service` callbacks.
- `features/feature_registry.c` selects the active module from `features/feature_config.h`.
- `features/oscilloscope/` owns scope configuration, ADC/DMA setup, trigger control, capture timing, bring-up diagnostics, and capture output.
- `features/usb_test/` owns the repeating USB transport test mode.
- A future logic analyzer belongs in `features/logic_analyzer/` and should implement the same module contract rather than expanding `main.c`.
- Generated PSoC component code remains outside the feature modules and is accessed only by the feature that owns the corresponding hardware path.

## Verified USB State

- USB CDC enumerates successfully.
- The live Windows device was observed as `USB Serial Device (COM10)`.
- Hardware ID: `USB\\VID_04B4&PID_F232`.
- The firmware product string is `CypressScope`.
- Windows reports `CypressScope` as the bus-reported device description, while the friendly name remains `USB Serial Device (COM10)` because Windows is using the generic `usbser.inf` CDC driver.
- `COM8` is the separate KitProg USB-UART and is not the application USB CDC device.
- Initial `cdc_not_ready` and `tx_timeout` messages during enumeration or port opening are expected. After the host opens the CDC port, repeated `DBG,USBTEST,send=ok` messages confirm that the application endpoint is working.
- The generated USB descriptor already contains the desired product string. Future descriptor changes should be made through the PSoC Creator USB component configuration, not by editing generated files directly.

## Current Firmware Modes

`USB_Test.cydsn/main.c` currently has:

- `BRINGUP_TEST = 1`
- `TIMING_DIAGNOSTICS = 1`
- `USB_TEST_MODE = 1`
- `USB_TEST_INTERVAL_MS = 1000`

With `USB_TEST_MODE` enabled, the firmware sends a synthetic, fixed 16-row ASCII test frame once per second. This verifies USB transport, framing, and host reception; it does not benchmark maximum USB throughput and it does not contain the live 1024-sample capture.

The test frame intentionally reports `META,PRE_TRIGGER,8` and `META,POST_TRIGGER,8` because it contains 16 synthetic rows. Those values do not describe the real capture configuration below.

## Current Capture Configuration

- Buffer: `1024` samples per channel.
- Pre-trigger: `274` samples.
- Post-trigger: `750` samples.
- ADC resolution: `12` bits.
- Trigger DAC levels: `156` for both channels.
- Timer UI period: `999`.
- Generated timer register value: `Timer_INIT_PERIOD = 998`.
- Effective timer period: `999` bus-clock cycles.
- Bus clock: `24,000,000 Hz`.
- Sample rate: `24,024 samples/sec/channel`.
- Sample period: `41.625 us` (`41,625 ns`).
- ADC0 and ADC1 clock: `1.6 MHz`.
- ADC sample precharge: `4` clocks.
- DMA0 and DMA1 write 16-bit ADC results into independent circular buffers.
- Each DMA transfer uses a 2-byte burst and increments the destination address.
- The current capture path gates DMA requests off after the post-trigger counter completes.

The timing values are calculated in `main.c` from the generated bus-clock and timer definitions. The UI's period value is therefore one greater than the generated counter/register value.

## Current Hardware Timing Path

- The separate `TimerClock` was removed.
- The trigger/sample clock component was renamed to `Clock` and uses the existing `BUS_CLK` source.
- The same 24 MHz bus-clock source is used for the timer and trigger synchronization path.
- `PWM_Sample_Buffer` is clocked directly from `ADC0` EOC, not from `BUS_CLK`.
- The trigger comparator/synchronizer path currently uses the rebuilt physical wiring, including the synchronized comparator output feeding the edge detector and trigger status path.
- The earlier 18 MHz trigger-clock warning belongs to the previous clock arrangement; the current generated design should be checked again after any schematic regeneration.

## Verified Capture Behavior

Previous hardware captures demonstrated the expected sequence:

1. Clear status: `0x00`.
2. Arm with control bit `0x40`.
3. Trigger status changes to `0x05` (`triggered + comparator 0`).
4. Capture-done status changes to `0x0D` (`comparator 0 + triggered + done`).
5. Both DMA channels report the same final index.
6. The post-trigger measurement reports exactly `750` samples.

The physical test setup used PWM on channel 0 and ground on channel 1. The expected raw ADC pattern is therefore channel 0 alternating between approximately `0` and `4095`, with channel 1 remaining at `0`. The observed output matches that pattern.

## Communication State

The application protocol is currently ASCII/UTF-8 with CRLF line endings:

```text
CAPTURE,BEGIN
META,...
TRIGGER_INDEX,...
IDX,...
DATA
<channel_0_count>,<channel_1_count>
...
CAPTURE,END
```

Current implementation details:

- `app_comm.c` owns the USB application transport and test frame.
- USB writes are split into 64-byte-or-smaller CDC transfers.
- USB transmit waits are bounded so a disconnected host does not block forever.
- `USBUART_CDC_Init()` is treated correctly: `USBUART_SUCCESS` is zero.
- A full host command/handshake path is not implemented yet.
- The dynamic capture sender already accepts capture metadata; the active test mode currently uses synthetic fixed data.

## Performance Baseline

At the current sample rate, storing two channels as 16-bit values produces approximately:

```text
2 channels * 2 bytes * 24,024 samples/sec = 96,096 raw bytes/sec
```

The current ASCII CSV representation requires substantially more bandwidth than the raw binary representation. The hardware capture path has kept up with the current rate in the observed tests, including an exact `750`-sample post-trigger measurement. The USB test loop only proves that the endpoint can repeatedly send the small synthetic frame; it is not a full-rate streaming test.

The current ADC and timer limits have not been characterized by an automated rate sweep. The generated ADC headers permit an ADC clock up to 18 MHz, but the practical maximum sample rate still depends on ADC conversion timing, SOC spacing, DMA arbitration, trigger behavior, and signal quality. Treat any rate above the current `24,024 samples/sec/channel` as unvalidated until measured.

## Planned Work

1. Replace ASCII capture payloads with framed binary bursts.
2. Add a versioned `HELLO`/`INFO` handshake and explicit capture commands.
3. Add host-side automatic device discovery using the CypressScope USB product identity, followed by protocol verification after opening the port.
4. Add an automated sample-rate sweep that checks ADC results, DMA indices, channel alignment, trigger timing, and missed samples.
5. Increase acquisition speed only after the sweep passes; keep ADC resolution and analog settling requirements explicit.
6. Add a logic analyzer mode with digital threshold/edge capture, packed channel data, trigger metadata, and binary output.
7. Add continuous or ping-pong buffering only if burst capture testing shows that it is required.
8. Update the application protocol document when the binary burst and logic analyzer formats are finalized.

## Important Caveats

- `USB_TEST_MODE` must be disabled before validating live capture data through USB.
- The current test frame's 8-sample pre/post values are synthetic and are not a timing error.
- Windows displaying `USB Serial Device (COM10)` is normal for the generic CDC driver and does not mean the product descriptor is missing.
- Generated PSoC source files are useful for verifying effective settings, but component configuration should be changed in PSoC Creator.
- The current design is optimized for correctness and observability, not maximum throughput. Burst transfer and speed optimization are deliberate next steps.

## Reference Files

- `USB_Test.cydsn/main.c` - capture setup, timing metadata, diagnostics, and test-mode selection.
- `USB_Test.cydsn/app_comm.c` - USB initialization, bounded CDC transfer, test frame, and capture framing.
- `USB_Test.cydsn/app_comm.h` - capture metadata structure and communication API.
- `USB_Test.cydsn/features/feature.h` - common feature-module interface.
- `USB_Test.cydsn/features/feature_config.h` - active feature selection.
- `USB_Test.cydsn/features/feature_registry.c` - active feature registry.
- `USB_Test.cydsn/features/oscilloscope/` - oscilloscope feature implementation.
- `USB_Test.cydsn/features/usb_test/` - USB transport test implementation.
- `USB_Test.cydsn/Generated_Source/PSoC5/Timer.h` - generated timer period.
- `USB_Test.cydsn/Generated_Source/PSoC5/ADC0.h` and `ADC1.h` - generated ADC timing settings.
- `USB_Test.cydsn/Generated_Source/PSoC5/USBUART_descr.c` - generated USB descriptor used to verify the product string.
- `Proper_Capture_Test.cydsn/APP_PROTOCOL.md` - earlier ASCII capture protocol reference.

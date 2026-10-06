# Capture USB Protocol

`DebugUART` is diagnostic-only. `USBUART` is the application link using USB
CDC. Firmware starts USB without waiting for a host; a capture is sent only
when CDC is configured and its transmit endpoint becomes ready.

## Capture frame

Each capture is UTF-8/ASCII text with CRLF line endings:

```text
CAPTURE,BEGIN
META,VERSION,1
META,CAPTURE_NUMBER,<n>
META,FORMAT,CSV16_ADC_COUNTS
META,CHANNEL_COUNT,2
META,SAMPLE_COUNT,1024
META,ADC_BITS,12
META,SAMPLE_CLOCK_HZ,24000000
META,TIMER_PERIOD_CYCLES,999
META,SAMPLE_RATE_HZ,24024
META,SAMPLE_PERIOD_NS,41625
META,PRE_TRIGGER,274
META,POST_TRIGGER,750
META,TRIGGER_INDEX,274
META,TRIGGER_OFFSET,0
META,TRIGGER0_LEVEL,156
META,TRIGGER1_LEVEL,156
META,STATUS,<status>
META,END_INDEX0,<index>
META,END_INDEX1,<index>
META,DATA_START_INDEX,<index>
META,DATA_ROWS,1024
TRIGGER_INDEX,274
IDX,<index0>,<index1>
DATA
<adc0>,<adc1>
...
CAPTURE,END
```

The two values after `DATA` are raw unsigned ADC counts. `TRIGGER_INDEX` is
the row where the trigger is presented after ring rotation. For row `r`, the
sample time relative to the trigger is:

```text
time_ns = (r - TRIGGER_INDEX) * SAMPLE_PERIOD_NS
```

The host should use `SAMPLE_CLOCK_HZ` and `TIMER_PERIOD_CYCLES` as the exact
timing source; `SAMPLE_RATE_HZ` is an integer convenience value.

`STATUS` uses the firmware status bits: bit 0 CMP0 level, bit 1 CMP1 level,
bit 2 triggered, and bit 3 capture complete. `IDX` is retained as a compact
summary for diagnostics and channel alignment checks.

The USB transport sends payloads in 64-byte-or-smaller CDC packets. Commands
from the application are not enabled yet; the next protocol revision can add
versioned control frames without changing the capture frame format.

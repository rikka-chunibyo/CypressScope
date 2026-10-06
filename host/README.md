# CypressScope Live Viewer

This small test application opens the USB CDC serial port, sends MODE LIVE,
parses the binary LIVE frames, and graphs both ADC channels.

## Install

    python -m pip install -r host\requirements.txt

## Run

    python host\live_viewer.py --list-ports
    python host\live_viewer.py COM10

The USB CDC baud rate is normally ignored by the device, but the viewer uses
115200 by default for serial-driver compatibility. Use --no-command if the
firmware is already streaming live data.

    python host\live_viewer.py COM10 --no-command --window 8192

## Protocol

Each frame starts with a 20-byte little-endian header:

| Bytes | Meaning |
| --- | --- |
| 0..3 | ASCII LIVE |
| 4 | Protocol version |
| 5 | Channel count |
| 6..7 | Sample count |
| 8..9 | Ring-buffer start index |
| 10..13 | Frame sequence |
| 14..17 | Sample rate in Hz |
| 18..19 | Payload length in bytes |

The payload contains rows of two unsigned 16-bit little-endian ADC values:
channel0, channel1. A terminal will display these binary bytes as garbage;
that is expected.

## Parser Test

Run the protocol parser without hardware:

    python host\live_viewer.py --self-test

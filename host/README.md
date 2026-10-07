# CypressScope Viewer

This small application opens the USB CDC serial port and graphs both ADC
channels. Trigger mode is the current test path; live mode remains available
for the future continuous binary stream.

## Install

    python -m pip install -r host\requirements.txt

## Run

    python host\viewer.py --list-ports
    python host\viewer.py COM10

The default mode sends `MODE SCOPE`, parses the triggered ASCII capture, and
plots the samples around the trigger index. Select the future binary stream
explicitly with:

    python host\viewer.py COM10 --mode live

In trigger mode, edit either trigger box in the plot and press Enter to send
the new VDAC code to the device. The range is `0` through `255`. Trigger
levels can also be set at startup:

    python host\viewer.py COM10 --trigger0 156 --trigger1 156

The USB CDC baud rate is normally ignored by the device, but the viewer uses
115200 by default for serial-driver compatibility. Use --no-command if the
firmware is already streaming live data.

    python host\viewer.py COM10 --mode live --no-command --window 8192

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

## Parser Tests

Run both protocol parsers without hardware:

    python host\viewer.py --self-test

## Commands

The firmware accepts these runtime trigger commands over USB CDC:

    TRIGGER 0 156
    TRIGGER 1 156

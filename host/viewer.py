"""Plot CypressScope LIVE frames received over a USB CDC serial port."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import struct
import sys
from typing import Deque, Dict, List, Optional, Sequence, Tuple


MAGIC = b"LIVE"
HEADER = struct.Struct("<4sBBHHIIH")
ROW = struct.Struct("<HH")
MAX_FRAME_SAMPLES = 4096
DEFAULT_TRIGGER_LEVEL = 156


@dataclass(frozen=True)
class LiveFrame:
    version: int
    channel_count: int
    sample_count: int
    start_index: int
    sequence: int
    sample_rate_hz: int
    channel0: Tuple[int, ...]
    channel1: Tuple[int, ...]


class LiveFrameParser:
    """Reassembles LIVE frames from arbitrary serial read boundaries."""

    def __init__(self, max_samples: int = MAX_FRAME_SAMPLES) -> None:
        self._buffer = bytearray()
        self.max_samples = max_samples
        self.frames_parsed = 0
        self.malformed_headers = 0
        self.bytes_discarded = 0

    def feed(self, data: bytes) -> List[LiveFrame]:
        self._buffer.extend(data)
        frames: List[LiveFrame] = []

        while True:
            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                keep = min(len(self._buffer), len(MAGIC) - 1)
                self.bytes_discarded += len(self._buffer) - keep
                if keep:
                    self._buffer[:] = self._buffer[-keep:]
                else:
                    self._buffer.clear()
                break

            if magic_index:
                self.bytes_discarded += magic_index
                del self._buffer[:magic_index]

            if len(self._buffer) < HEADER.size:
                break

            (
                magic,
                version,
                channel_count,
                sample_count,
                start_index,
                sequence,
                sample_rate_hz,
                payload_length,
            ) = HEADER.unpack_from(self._buffer)

            expected_payload_length = sample_count * ROW.size
            if (
                magic != MAGIC
                or version != 1
                or channel_count != 2
                or sample_count == 0
                or sample_count > self.max_samples
                or payload_length != expected_payload_length
            ):
                self.malformed_headers += 1
                del self._buffer[0]
                continue

            frame_length = HEADER.size + payload_length
            if len(self._buffer) < frame_length:
                break

            payload = self._buffer[HEADER.size:frame_length]
            channel0 = []
            channel1 = []
            for offset in range(0, len(payload), ROW.size):
                value0, value1 = ROW.unpack_from(payload, offset)
                channel0.append(value0)
                channel1.append(value1)

            frames.append(
                LiveFrame(
                    version=version,
                    channel_count=channel_count,
                    sample_count=sample_count,
                    start_index=start_index,
                    sequence=sequence,
                    sample_rate_hz=sample_rate_hz,
                    channel0=tuple(channel0),
                    channel1=tuple(channel1),
                )
            )
            self.frames_parsed += 1
            del self._buffer[:frame_length]

        return frames


@dataclass(frozen=True)
class TriggerCapture:
    metadata: Dict[str, str]
    channel0: Tuple[int, ...]
    channel1: Tuple[int, ...]

    @property
    def sample_rate_hz(self) -> int:
        return max(int(self.metadata.get("SAMPLE_RATE_HZ", "1")), 1)

    @property
    def trigger_index(self) -> int:
        return int(self.metadata.get("TRIGGER_INDEX", "0"))


class TriggerCaptureParser:
    """Reassembles ASCII triggered captures from arbitrary serial reads."""

    def __init__(self, max_samples: int = MAX_FRAME_SAMPLES) -> None:
        self._buffer = bytearray()
        self._in_capture = False
        self._in_data = False
        self._metadata: Dict[str, str] = {}
        self._channel0: List[int] = []
        self._channel1: List[int] = []
        self.max_samples = max_samples
        self.captures_parsed = 0
        self.malformed_lines = 0

    def feed(self, data: bytes) -> List[TriggerCapture]:
        self._buffer.extend(data)
        captures: List[TriggerCapture] = []

        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                break

            raw_line = bytes(self._buffer[:newline]).rstrip(b"\r")
            del self._buffer[:newline + 1]
            line = raw_line.decode("ascii", errors="replace").strip()
            capture = self._process_line(line)
            if capture is not None:
                captures.append(capture)

        return captures

    def _start_capture(self) -> None:
        self._in_capture = True
        self._in_data = False
        self._metadata = {}
        self._channel0 = []
        self._channel1 = []

    def _finish_capture(self) -> Optional[TriggerCapture]:
        expected = int(self._metadata.get("SAMPLE_COUNT", str(len(self._channel0))))
        valid = (
            0 < len(self._channel0) <= self.max_samples
            and len(self._channel0) == len(self._channel1)
            and len(self._channel0) == expected
        )
        capture: Optional[TriggerCapture] = None
        if valid:
            capture = TriggerCapture(
                metadata=dict(self._metadata),
                channel0=tuple(self._channel0),
                channel1=tuple(self._channel1),
            )
            self.captures_parsed += 1
        else:
            self.malformed_lines += 1

        self._in_capture = False
        self._in_data = False
        self._metadata = {}
        self._channel0 = []
        self._channel1 = []
        return capture

    def _process_line(self, line: str) -> Optional[TriggerCapture]:
        if line == "CAPTURE,BEGIN":
            self._start_capture()
            return None

        if not self._in_capture:
            return None

        if line == "CAPTURE,END":
            return self._finish_capture()

        if line == "DATA":
            self._in_data = True
            return None

        if line.startswith("META,"):
            fields = line.split(",", 2)
            if len(fields) == 3:
                self._metadata[fields[1]] = fields[2]
            else:
                self.malformed_lines += 1
            return None

        if line.startswith("TRIGGER_INDEX,") or line.startswith("IDX,"):
            return None

        if self._in_data:
            fields = line.split(",")
            if len(fields) == 2:
                try:
                    value0 = int(fields[0], 10)
                    value1 = int(fields[1], 10)
                except ValueError:
                    value0 = -1
                    value1 = -1

                if 0 <= value0 <= 0xFFFF and 0 <= value1 <= 0xFFFF:
                    if len(self._channel0) < self.max_samples:
                        self._channel0.append(value0)
                        self._channel1.append(value1)
                        return None

            self.malformed_lines += 1

        return None


class SerialReceiver:
    def __init__(self, port: str, baudrate: int, parser: LiveFrameParser) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required; install host\\requirements.txt"
            ) from exc

        self._serial_module = serial
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0,
            write_timeout=1,
        )
        self.parser = parser

    def send_command(self, command: str) -> None:
        command = command.rstrip("\r\n") + "\r\n"
        self.serial.reset_input_buffer()
        self.serial.write(command.encode("ascii"))
        self.serial.flush()

    def read_bytes(self) -> bytes:
        waiting = self.serial.in_waiting
        if waiting == 0:
            return b""
        return self.serial.read(min(waiting, 65536))

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()


def run_parser_self_test() -> None:
    values0 = (0, 1024, 2048, 4095)
    values1 = (4095, 2048, 1024, 0)
    payload = b"".join(
        ROW.pack(value0, value1) for value0, value1 in zip(values0, values1)
    )
    packet = HEADER.pack(
        MAGIC,
        1,
        2,
        len(values0),
        17,
        23,
        24024,
        len(payload),
    ) + payload

    parser = LiveFrameParser()
    frames = []
    for start in range(0, len(packet), 3):
        frames.extend(parser.feed(packet[start:start + 3]))

    assert len(frames) == 1
    assert frames[0].sequence == 23
    assert frames[0].start_index == 17
    assert frames[0].channel0 == values0
    assert frames[0].channel1 == values1
    print("LIVE parser self-test passed")


def run_trigger_parser_self_test() -> None:
    packet = (
        b"ignored\r\n"
        b"CAPTURE,BEGIN\r\n"
        b"META,SAMPLE_COUNT,4\r\n"
        b"META,SAMPLE_RATE_HZ,24024\r\n"
        b"META,TRIGGER_INDEX,2\r\n"
        b"DATA\r\n"
        b"1,11\r\n"
        b"2,12\r\n"
        b"3,13\r\n"
        b"4,14\r\n"
        b"CAPTURE,END\r\n"
    )

    parser = TriggerCaptureParser()
    captures: List[TriggerCapture] = []
    for start in range(0, len(packet), 5):
        captures.extend(parser.feed(packet[start:start + 5]))

    assert len(captures) == 1
    assert captures[0].sample_rate_hz == 24024
    assert captures[0].trigger_index == 2
    assert captures[0].channel0 == (1, 2, 3, 4)
    assert captures[0].channel1 == (11, 12, 13, 14)
    print("TRIGGER parser self-test passed")


def list_serial_ports() -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is required; install host\\requirements.txt", file=sys.stderr)
        return 2

    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return 0

    for port in ports:
        description = port.description or "unknown device"
        print(f"{port.device}: {description}")
    return 0


def run_viewer(args: argparse.Namespace) -> int:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import TextBox
    except ImportError:
        print(
            "matplotlib is required; install host\\requirements.txt",
            file=sys.stderr,
        )
        return 2

    live_parser = LiveFrameParser()
    trigger_parser = TriggerCaptureParser()
    command = args.command
    if command is None:
        command = "MODE SCOPE" if args.mode == "trigger" else "MODE LIVE"

    try:
        receiver = SerialReceiver(args.port, args.baudrate, live_parser)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        return 2

    if not args.no_command:
        try:
            receiver.send_command(command)
        except receiver._serial_module.SerialException as exc:
            receiver.close()
            print(f"Unable to send {command!r}: {exc}", file=sys.stderr)
            return 2

    if args.mode == "trigger":
        for channel, level in ((0, args.trigger0), (1, args.trigger1)):
            if level is None:
                continue
            try:
                receiver.send_command(f"TRIGGER {channel} {level}")
            except receiver._serial_module.SerialException as exc:
                receiver.close()
                print(f"Unable to set trigger {channel} to {level}: {exc}", file=sys.stderr)
                return 2

    times: Deque[float] = deque(maxlen=args.window)
    channel0: Deque[int] = deque(maxlen=args.window)
    channel1: Deque[int] = deque(maxlen=args.window)
    sample_rate_hz = 1
    sample_position = 0
    last_sequence: Optional[int] = None
    dropped_frames = 0
    received_samples = 0
    received_captures = 0

    figure, axes = plt.subplots(figsize=(11, 6))
    line0, = axes.plot([], [], label="ADC0", color="tab:blue")
    line1, = axes.plot([], [], label="ADC1", color="tab:orange")
    status = axes.text(
        0.01,
        0.98,
        "Waiting for triggered captures..." if args.mode == "trigger" else "Waiting for LIVE frames...",
        transform=axes.transAxes,
        va="top",
        family="monospace",
    )
    axes.set_title(
        f"CypressScope {args.mode} viewer - {args.port}"
    )
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("ADC counts")
    axes.set_ylim(args.adc_min, args.adc_max)
    axes.grid(True, alpha=0.3)
    axes.legend(loc="upper right")

    trigger_message = ""
    if args.mode == "trigger":
        figure.subplots_adjust(bottom=0.23)
        trigger0_axes = figure.add_axes([0.10, 0.11, 0.28, 0.055])
        trigger1_axes = figure.add_axes([0.52, 0.11, 0.28, 0.055])
        trigger0_box = TextBox(
            trigger0_axes,
            "Trigger 0 VDAC: ",
            initial=str(args.trigger0 if args.trigger0 is not None else DEFAULT_TRIGGER_LEVEL),
        )
        trigger1_box = TextBox(
            trigger1_axes,
            "Trigger 1 VDAC: ",
            initial=str(args.trigger1 if args.trigger1 is not None else DEFAULT_TRIGGER_LEVEL),
        )
        figure.text(
            0.10,
            0.04,
            "Enter a value from 0 to 255 and press Enter. Changes apply to the next capture.",
            family="monospace",
        )

        def submit_trigger(channel: int, text: str) -> None:
            nonlocal trigger_message

            try:
                level = int(text.strip(), 10)
            except ValueError:
                trigger_message = f"invalid trigger {channel}"
                status.set_text(trigger_message)
                return

            if not 0 <= level <= 255:
                trigger_message = f"trigger {channel} must be 0..255"
                status.set_text(trigger_message)
                return

            try:
                receiver.send_command(f"TRIGGER {channel} {level}")
            except receiver._serial_module.SerialException as exc:
                trigger_message = f"trigger {channel} error: {exc}"
            else:
                trigger_message = f"trigger{channel}={level} sent"
            status.set_text(trigger_message)

        trigger0_box.on_submit(lambda text: submit_trigger(0, text))
        trigger1_box.on_submit(lambda text: submit_trigger(1, text))

    def update(_frame_number: int):
        nonlocal sample_rate_hz
        nonlocal sample_position
        nonlocal last_sequence
        nonlocal dropped_frames
        nonlocal received_samples
        nonlocal received_captures

        try:
            data = receiver.read_bytes()
        except receiver._serial_module.SerialException as exc:
            status.set_text(f"Serial error: {exc}")
            return line0, line1, status

        if args.mode == "trigger":
            captures = trigger_parser.feed(data)
            if captures:
                capture = captures[-1]
                received_captures += len(captures)
                sample_rate_hz = capture.sample_rate_hz
                trigger_index = capture.trigger_index
                sample_count = len(capture.channel0)
                start = 0
                end = sample_count
                if sample_count > args.window:
                    start = max(0, min(trigger_index - (args.window // 2), sample_count - args.window))
                    end = start + args.window

                x_values = [
                    (index - trigger_index) / sample_rate_hz
                    for index in range(start, end)
                ]
                line0.set_data(x_values, capture.channel0[start:end])
                line1.set_data(x_values, capture.channel1[start:end])
                if x_values:
                    axes.set_xlim(x_values[0], x_values[-1])

                if args.auto_scale:
                    values = list(capture.channel0[start:end]) + list(capture.channel1[start:end])
                    low = min(values)
                    high = max(values)
                    padding = max(25.0, (high - low) * 0.1)
                    axes.set_ylim(
                        max(args.adc_min, low - padding),
                        min(args.adc_max, high + padding),
                    )

                status_value = capture.metadata.get("STATUS", "?")
                status.set_text(
                    f"captures={received_captures} samples={sample_count} "
                    f"rate={sample_rate_hz}Hz trigger={trigger_index} "
                    f"status={status_value} bad={trigger_parser.malformed_lines}"
                    + (f" {trigger_message}" if trigger_message else "")
                )
        else:
            frames = live_parser.feed(data)
            for frame in frames:
                if last_sequence is not None:
                    expected = (last_sequence + 1) & 0xFFFFFFFF
                    if frame.sequence != expected:
                        dropped_frames += (frame.sequence - expected) & 0xFFFFFFFF
                last_sequence = frame.sequence
                sample_rate_hz = max(frame.sample_rate_hz, 1)

                for value0, value1 in zip(frame.channel0, frame.channel1):
                    times.append(sample_position / sample_rate_hz)
                    channel0.append(value0)
                    channel1.append(value1)
                    sample_position += 1
                received_samples += frame.sample_count

        if args.mode == "live" and times:
            x_values = list(times)
            line0.set_data(x_values, list(channel0))
            line1.set_data(x_values, list(channel1))

            span = args.window / sample_rate_hz
            right = x_values[-1]
            left = max(0.0, right - span)
            axes.set_xlim(left, max(right, span))

            if args.auto_scale:
                values = list(channel0) + list(channel1)
                low = min(values)
                high = max(values)
                padding = max(25.0, (high - low) * 0.1)
                axes.set_ylim(
                    max(args.adc_min, low - padding),
                    min(args.adc_max, high + padding),
                )

            status.set_text(
                f"frames={live_parser.frames_parsed} samples={received_samples} "
                f"rate={sample_rate_hz}Hz seq={last_sequence} "
                f"dropped={dropped_frames} bad={live_parser.malformed_headers}"
            )

        return line0, line1, status

    def close(_event) -> None:
        receiver.close()

    figure.canvas.mpl_connect("close_event", close)
    animation = FuncAnimation(
        figure,
        update,
        interval=args.refresh_ms,
        blit=False,
        cache_frame_data=False,
    )
    figure._cypressscope_animation = animation

    try:
        plt.show()
    finally:
        receiver.close()
    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Graph CypressScope triggered captures or binary LIVE frames."
    )
    parser.add_argument("port", nargs="?", help="serial port, for example COM10")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument(
        "--mode",
        choices=("trigger", "live"),
        default="trigger",
        help="device stream to display (default: trigger)",
    )
    parser.add_argument("--window", type=int, default=4096)
    parser.add_argument("--refresh-ms", type=int, default=50)
    parser.add_argument(
        "--command",
        default=None,
        help="override the command sent after opening the port",
    )
    parser.add_argument("--no-command", action="store_true")
    parser.add_argument(
        "--trigger0",
        type=int,
        default=None,
        help="set channel 0 trigger VDAC code before capturing (0..255)",
    )
    parser.add_argument(
        "--trigger1",
        type=int,
        default=None,
        help="set channel 1 trigger VDAC code before capturing (0..255)",
    )
    parser.add_argument("--adc-min", type=float, default=0)
    parser.add_argument("--adc-max", type=float, default=4095)
    parser.add_argument("--auto-scale", action="store_true")
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    if args.self_test:
        run_parser_self_test()
        run_trigger_parser_self_test()
        return 0
    if args.list_ports:
        return list_serial_ports()
    if not args.port:
        parser.error("a serial port is required unless --list-ports or --self-test is used")
    if args.window <= 0 or args.refresh_ms <= 0:
        parser.error("--window and --refresh-ms must be positive")
    if args.adc_max <= args.adc_min:
        parser.error("--adc-max must be greater than --adc-min")
    for name in ("trigger0", "trigger1"):
        value = getattr(args, name)
        if value is not None and not 0 <= value <= 255:
            parser.error(f"--{name} must be between 0 and 255")

    return run_viewer(args)


if __name__ == "__main__":
    raise SystemExit(main())

"""Plot CypressScope LIVE frames received over a USB CDC serial port."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import struct
import sys
from typing import Deque, List, Optional, Sequence, Tuple


MAGIC = b"LIVE"
HEADER = struct.Struct("<4sBBHHIIH")
ROW = struct.Struct("<HH")
MAX_FRAME_SAMPLES = 4096


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


class SerialLiveReceiver:
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

    def read_frames(self) -> List[LiveFrame]:
        waiting = self.serial.in_waiting
        if waiting == 0:
            return []
        return self.parser.feed(self.serial.read(min(waiting, 65536)))

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
    except ImportError:
        print(
            "matplotlib is required; install host\\requirements.txt",
            file=sys.stderr,
        )
        return 2

    parser = LiveFrameParser()
    try:
        receiver = SerialLiveReceiver(args.port, args.baudrate, parser)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        return 2

    if not args.no_command:
        try:
            receiver.send_command(args.command)
        except receiver._serial_module.SerialException as exc:
            receiver.close()
            print(f"Unable to send {args.command!r}: {exc}", file=sys.stderr)
            return 2

    times: Deque[float] = deque(maxlen=args.window)
    channel0: Deque[int] = deque(maxlen=args.window)
    channel1: Deque[int] = deque(maxlen=args.window)
    sample_rate_hz = 1
    sample_position = 0
    last_sequence: Optional[int] = None
    dropped_frames = 0
    received_samples = 0

    figure, axes = plt.subplots(figsize=(11, 6))
    line0, = axes.plot([], [], label="ADC0", color="tab:blue")
    line1, = axes.plot([], [], label="ADC1", color="tab:orange")
    status = axes.text(
        0.01,
        0.98,
        "Waiting for LIVE frames...",
        transform=axes.transAxes,
        va="top",
        family="monospace",
    )
    axes.set_title(f"CypressScope live view - {args.port}")
    axes.set_xlabel("Time (s)")
    axes.set_ylabel("ADC counts")
    axes.set_ylim(args.adc_min, args.adc_max)
    axes.grid(True, alpha=0.3)
    axes.legend(loc="upper right")

    def update(_frame_number: int):
        nonlocal sample_rate_hz
        nonlocal sample_position
        nonlocal last_sequence
        nonlocal dropped_frames
        nonlocal received_samples

        try:
            frames = receiver.read_frames()
        except receiver._serial_module.SerialException as exc:
            status.set_text(f"Serial error: {exc}")
            return line0, line1, status

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

        if times:
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
                f"frames={parser.frames_parsed} samples={received_samples} "
                f"rate={sample_rate_hz}Hz seq={last_sequence} "
                f"dropped={dropped_frames} bad={parser.malformed_headers}"
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
        description="Graph CypressScope binary LIVE frames."
    )
    parser.add_argument("port", nargs="?", help="serial port, for example COM10")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--window", type=int, default=4096)
    parser.add_argument("--refresh-ms", type=int, default=50)
    parser.add_argument("--command", default="MODE LIVE")
    parser.add_argument("--no-command", action="store_true")
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
        return 0
    if args.list_ports:
        return list_serial_ports()
    if not args.port:
        parser.error("a serial port is required unless --list-ports or --self-test is used")
    if args.window <= 0 or args.refresh_ms <= 0:
        parser.error("--window and --refresh-ms must be positive")
    if args.adc_max <= args.adc_min:
        parser.error("--adc-max must be greater than --adc-min")

    return run_viewer(args)


if __name__ == "__main__":
    raise SystemExit(main())

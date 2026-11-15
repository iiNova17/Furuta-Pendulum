"""Record a bounded USB bench observation with explicit command timestamps.

No DTR/RTS reset, no input-buffer discard, and a final stop before closing.
The firmware's command set has no motor-enable operation.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import time

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--directory", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--send", action="append", default=[],
                        help="seconds:single_command, repeatable")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.label):
        parser.error("label must be a simple filename")
    if not 1 <= args.seconds <= 300:
        parser.error("duration must be 1..300 seconds")
    schedule = [(0.4, "s")]
    for item in args.send:
        at, command = item.split(":", 1)
        if command not in ("h", "b", "p", "i", "a", "e", "z", "t", "s", "x"):
            parser.error("unknown firmware command")
        at = float(at)
        if not 0 <= at < args.seconds:
            parser.error("command time outside capture interval")
        schedule.append((at, command))
    schedule.sort()
    args.directory.mkdir(parents=True, exist_ok=True)
    path = args.directory / args.label
    # Never overwrite an existing observation.
    for suffix in (".jsonl", ".raw", ".summary.json"):
        if path.with_suffix(suffix).exists():
            parser.error(f"capture already exists: {path}")
    started = time.monotonic()
    lines = []
    synchronized = False
    stop_sent = False
    pending = bytearray()
    port = serial.Serial()
    port.port = args.port
    port.baudrate = 115200
    port.timeout = 0.1
    port.write_timeout = 1
    port.dtr = False
    port.rts = False
    with path.with_suffix(".jsonl").open("x", encoding="utf-8") as events, \
            path.with_suffix(".raw").open("xb") as raw:
        def record(kind, **data):
            item = {"utc": datetime.now(timezone.utc).isoformat(),
                    "elapsed_s": round(time.monotonic() - started, 3),
                    "kind": kind, **data}
            events.write(json.dumps(item) + "\n")
            events.flush()

        def send(command):
            nonlocal stop_sent
            port.write(command.encode("ascii"))
            port.flush()
            if command == "s":
                stop_sent = True
            record("send", command=command)
            print(f"SEND {command} at {time.monotonic() - started:.1f}s", flush=True)

        def receive():
            nonlocal synchronized
            data = port.read(max(1, min(port.in_waiting, 8192)))
            if not data:
                return
            raw.write(data)
            raw.flush()
            pending.extend(data)
            while b"\n" in pending:
                line, _, rest = pending.partition(b"\n")
                pending[:] = rest
                text = line.decode("utf-8", errors="replace").rstrip("\r")
                record("line", text=text)
                if stop_sent and text.startswith("STOP:"):
                    synchronized = True
                # Reopened FTDI ports can deliver buffered fragments. Preserve all
                # bytes/events, but summarize only after the fresh stop response.
                if synchronized:
                    lines.append(text)
                # Keep raw telemetry in the files; show commands and diagnostics live.
                if synchronized and text.startswith((
                    "STOP:", "ZERO ", "Need >=", "Motor count", "Pendulum ",
                    "Angle zero", "Furuta fresh", "Chip=", "Free heap=",
                    "UART0/", "ADC=", "No motor", "I2C scan", "  ACK",
                    "Scan ", "  Bus error", "LOGIC", "FATAL",
                )):
                    print(text, flush=True)

        try:
            port.open()
            record("open", port=args.port, baud=115200, label=args.label,
                   dtr=False, rts=False)
            print(f"CAPTURE ACTIVE: {args.label}, {args.seconds:g} seconds", flush=True)
            while time.monotonic() - started < args.seconds:
                while schedule and time.monotonic() - started >= schedule[0][0]:
                    _, command = schedule.pop(0)
                    send(command)
                receive()
        finally:
            if port.is_open:
                try:
                    send("s")
                    deadline = time.monotonic() + 1
                    while time.monotonic() < deadline:
                        receive()
                finally:
                    port.close()
                    record("close")
            if pending:
                record("partial_line", text=pending.decode("utf-8", errors="replace"))

    encoder = []
    acs = []
    angles = []
    currents = []
    for line in lines:
        match = re.match(r"\[\d+ ms\] STBY_cmd=0 logic=\w+ ENC=(-?\d+) cps=([-.\d]+) A/B=(\d)/(\d)", line)
        if match:
            encoder.append((int(match[1]), float(match[2]), match[3] + "/" + match[4]))
        match = re.match(r"ACS: ([.\d]+) mV SD=([.\d]+) span=(\d+)\.\.(\d+) mV raw=(\d+)\.\.(\d+) N=(\d+)", line)
        if match:
            acs.append(tuple(float(match[i]) for i in range(1, 8)))
        match = re.match(r"AS5600: raw=(\d+) angle=([.\d]+) deg MD=(\d) ML=(\d) MH=(\d) AGC=(\d+) MAG=(\d+)", line)
        if match:
            angles.append(tuple(float(match[i]) for i in range(1, 8)))
        match = re.search(r"I_est=([-.\d]+) A", line)
        if match:
            currents.append(float(match[1]))

    def limits(values):
        return {"min": min(values), "max": max(values),
                "mean": sum(values) / len(values)} if values else None

    summary = {
        "label": args.label,
        "port": args.port,
        "encoder_reports": len(encoder),
        "encoder_counts": limits([r[0] for r in encoder]),
        "encoder_cps": limits([r[1] for r in encoder]),
        "encoder_levels": sorted(set(r[2] for r in encoder)),
        "acs_reports": len(acs),
        "acs_report_mean_mv": limits([r[0] for r in acs]),
        "acs_sample_sd_mv": limits([r[1] for r in acs]),
        "acs_raw_min": min((r[4] for r in acs), default=None),
        "acs_raw_max": max((r[5] for r in acs), default=None),
        "estimated_current_a": limits(currents),
        "pendulum_reports": len(angles),
        "pendulum_raw": limits([r[0] for r in angles]),
        "magnet_statuses_md_ml_mh": sorted(set(tuple(r[2:5]) for r in angles)),
        "pendulum_read_error_lines": sum("AS5600: READ ERROR" in r for r in lines),
        "events": [r for r in lines if any(token in r for token in
                    ("ZERO ", "zeroed", "position is zero", "refused",
                     "ACK ", "Scan complete", "Bus error", "LOGIC", "STOP:",
                     "Furuta fresh", "Chip=", "Free heap=", "FATAL", "rst:"))],
        "last_telemetry": [r for r in lines if r.startswith(("[", "ACS:", "AS5600:"))][-6:]
    }
    path.with_suffix(".summary.json").write_text(json.dumps(summary, indent=2),
                                                encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()

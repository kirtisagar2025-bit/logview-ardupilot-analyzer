"""Streaming ArduPilot DataFlash parsing and printable flight-test PDF reports."""
from __future__ import annotations

import math
import os
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from datetime import timedelta
from io import BytesIO
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape

import pandas as pd
from pymavlink import mavutil

MAX_UPLOAD_BYTES = 3 * 1024**3
MAX_SAMPLE_ROWS = 12_000
COPY_CHUNK_BYTES = 16 * 1024**2
SAMPLE_FIELDS = {
    "ATT": ["Roll", "Pitch", "Yaw", "DesRoll", "DesPitch", "DesYaw", "ErrRP", "ErrYaw"],
    "AHR2": ["Roll", "Pitch", "Yaw", "Alt", "Lat", "Lng"],
    "BAT": ["Volt", "VoltR", "Curr", "CurrTot", "EnrgTot", "RemPct", "Temp"],
    "BARO": ["Alt", "Temp", "Press", "Health"],
    "CTUN": ["Alt", "DAlt", "ThO", "ThH"],
    "GPS": ["I", "Status", "NSats", "HDop", "Lat", "Lng", "Alt", "Spd", "GMS", "GWk"],
    "IMU": ["I", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ", "Health"],
    "IMU2": ["I", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ", "Health"],
    "IMU3": ["I", "AccX", "AccY", "AccZ", "GyrX", "GyrY", "GyrZ", "Health"],
    "MAG": ["I", "MagX", "MagY", "MagZ", "OfsX", "OfsY", "OfsZ", "Health", "MOX", "MOY", "MOZ", "UT"],
    "XKF4": ["C", "SM", "SV", "SP", "SH", "SVT", "errRP", "OFN"],
    "NKF4": ["C", "SM", "SV", "SP", "SH", "SVT", "errRP", "OFN"],
    "RCIN": [f"C{i}" for i in range(1, 17)],
    "RCOU": [f"C{i}" for i in range(1, 17)],
    "VIBE": ["VibeX", "VibeY", "VibeZ", "Clip"],
    "MODE": ["Mode", "ModeNum", "ModeName", "ModeReason"],
}
EVENT_NAMES = {
    10: "ARMED", 11: "DISARMED", 15: "AUTO_ARMED", 17: "LAND_COMPLETE_MAYBE",
    18: "LAND_COMPLETE", 19: "LOST_GPS", 21: "FLIP_START", 22: "FLIP_END",
    25: "SET_HOME", 28: "NOT_LANDED", 41: "FENCE_ENABLE", 42: "FENCE_DISABLE",
}


class SampleBuffer:
    """Keep a bounded, evenly thinned stream of rows for plotting."""
    def __init__(self, limit: int = MAX_SAMPLE_ROWS):
        self.limit = limit
        self.rows: list[dict[str, Any]] = []
        self.count = 0
        self.stride = 1

    def add(self, row: dict[str, Any]) -> None:
        self.count += 1
        if self.count % self.stride:
            return
        self.rows.append(row)
        if len(self.rows) > self.limit:
            self.rows = self.rows[::2]
            self.stride *= 2


@dataclass
class NumericStats:
    count: int = 0
    total: float = 0.0
    minimum: float = math.inf
    maximum: float = -math.inf
    first: float | None = None
    last: float | None = None

    def add(self, value: Any) -> None:
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            return
        if not math.isfinite(number):
            return
        self.count += 1
        self.total += number
        self.minimum = min(self.minimum, number)
        self.maximum = max(self.maximum, number)
        if self.first is None:
            self.first = number
        self.last = number

    @property
    def average(self) -> float | None:
        return self.total / self.count if self.count else None


def copy_upload_to_temp(uploaded_file: Any, temp_dir: Path) -> str:
    """Copy Streamlit's file-like upload to an app-controlled disk file in chunks."""
    size = int(getattr(uploaded_file, "size", 0))
    if size <= 0:
        raise ValueError("The uploaded file is empty.")
    if size > MAX_UPLOAD_BYTES:
        raise ValueError("The maximum supported log size is 3 GB.")
    temp_dir.mkdir(parents=True, exist_ok=True)
    fd, path = tempfile.mkstemp(suffix=".bin", prefix="ardupilot-", dir=temp_dir)
    try:
        with os.fdopen(fd, "wb") as target:
            view = uploaded_file.getbuffer()
            try:
                for offset in range(0, len(view), COPY_CHUNK_BYTES):
                    target.write(view[offset:offset + COPY_CHUNK_BYTES])
            finally:
                del view
    except Exception:
        Path(path).unlink(missing_ok=True)
        raise
    return path


def parse_bin(path: str) -> dict[str, Any]:
    """Parse a BIN file incrementally; bound chart samples and aggregate statistics."""
    samples: dict[str, SampleBuffer] = {}
    counts: dict[str, int] = defaultdict(int)
    field_names: dict[str, list[str]] = {}
    numeric_stats: dict[str, dict[str, NumericStats]] = defaultdict(dict)
    events: list[dict[str, Any]] = []
    event_counts = 0
    log = None
    first_timestamp: float | None = None
    time_field: str | None = None
    time_scale = 1.0
    latest_elapsed = 0.0
    firmware = "Unknown"
    try:
        log = mavutil.mavlink_connection(path, dialect="ardupilotmega")
        while True:
            msg = log.recv_match(blocking=False)
            if msg is None:
                break
            kind = msg.get_type()
            if kind == "BAD_DATA":
                continue
            counts[kind] += 1
            wanted = SAMPLE_FIELDS.get(kind, [])
            is_event = kind in {"EV", "ERR", "MSG", "MODE"} and event_counts < 100_000
            decode_row = bool(wanted) or is_event or kind == "VER" or kind not in field_names
            row = msg.to_dict() if decode_row else {}
            row.pop("mavpackettype", None)
            if kind not in field_names:
                field_names[kind] = list(row.keys())

            if time_field is None:
                if "TimeUS" in row or hasattr(msg, "TimeUS"):
                    time_field, time_scale = "TimeUS", 1_000_000.0
                elif "TimeMS" in row or hasattr(msg, "TimeMS"):
                    time_field, time_scale = "TimeMS", 1_000.0
            raw_time = row.get("TimeUS", row.get("TimeMS"))
            if raw_time is None and time_field:
                raw_time = getattr(msg, time_field, None)
            elapsed = None
            if raw_time is not None:
                try:
                    raw_number = float(raw_time)
                    if first_timestamp is None:
                        first_timestamp = raw_number
                    elapsed = max(0.0, (raw_number - first_timestamp) / time_scale)
                    latest_elapsed = max(latest_elapsed, elapsed)
                except (TypeError, ValueError):
                    pass

            if wanted:
                sample_row = {key: row.get(key) for key in wanted if key in row}
                sample_row["Elapsed (s)"] = elapsed
                samples.setdefault(kind, SampleBuffer()).add(sample_row)
                for key in wanted:
                    if key in row:
                        numeric_stats[kind].setdefault(key, NumericStats()).add(row[key])
                if kind == "BAT" and row.get("Volt") is not None and row.get("Curr") is not None:
                    try:
                        power = float(row["Volt"]) * float(row["Curr"])
                        numeric_stats[kind].setdefault("Power", NumericStats()).add(power)
                    except (TypeError, ValueError):
                        pass

            if is_event:
                event_counts += 1
                event = dict(row)
                event["_type"] = kind
                event["Elapsed (s)"] = elapsed
                if kind == "EV":
                    try:
                        event["Event"] = EVENT_NAMES.get(int(row.get("Id")), f"EVENT {row.get('Id')}")
                    except (TypeError, ValueError):
                        event["Event"] = f"EVENT {row.get('Id', 'unknown')}"
                events.append(event)

            if kind == "VER":
                firmware = str(row.get("FWS", row.get("FW", firmware)))
            elif kind == "MSG" and firmware == "Unknown":
                line = str(row.get("Message", row.get("Msg", "")))
                if "Ardu" in line:
                    firmware = line[:160]
    finally:
        if log is not None:
            log.close()

    frames = {
        name: pd.DataFrame(buffer.rows)
        for name, buffer in samples.items() if buffer.rows
    }
    inventory = [
        {"Message": name, "Rows": count, "Fields": ", ".join(field_names.get(name, []))}
        for name, count in sorted(counts.items())
    ]
    if not counts:
        raise ValueError("No DataFlash messages were found. Check that this is a valid ArduPilot .bin log.")
    return {
        "frames": frames,
        "counts": dict(counts),
        "inventory": inventory,
        "events": events,
        "stats": numeric_stats,
        "duration_s": latest_elapsed,
        "firmware": firmware,
    }


def stat_value(parsed: dict[str, Any], message: str, field: str, key: str = "average") -> float | None:
    stat = parsed["stats"].get(message, {}).get(field)
    if stat is None or not stat.count:
        return None
    if key == "count":
        return float(stat.count)
    return getattr(stat, key)


def _number(value: float | None, digits: int = 2, suffix: str = "") -> str:
    return "not recorded" if value is None else f"{value:.{digits}f}{suffix}"


def _elapsed_text(seconds: float | None) -> str:
    if seconds is None:
        return "time unavailable"
    value = max(0, int(seconds))
    return str(timedelta(seconds=value))


def flight_intervals(parsed: dict[str, Any]) -> list[dict[str, Any]]:
    intervals: list[dict[str, Any]] = []
    active: float | None = None
    for event in parsed["events"]:
        if event.get("_type") != "EV":
            continue
        try:
            event_id = int(event.get("Id"))
        except (TypeError, ValueError):
            continue
        t = event.get("Elapsed (s)")
        if t is None:
            continue
        if event_id == 10:
            active = float(t)
        elif event_id == 11 and active is not None:
            end = float(t)
            intervals.append({"arm": active, "disarm": end, "duration": max(0.0, end - active)})
            active = None
    if active is not None:
        intervals.append({"arm": active, "disarm": None, "duration": max(0.0, parsed["duration_s"] - active)})
    return intervals


def get_summary(parsed: dict[str, Any], filename: str) -> tuple[list[dict[str, str]], dict[str, Any]]:
    stats = parsed["stats"]
    notes: list[dict[str, str]] = []
    metrics: dict[str, Any] = {"duration_s": parsed["duration_s"]}
    notes.append({"level": "info", "text": f"Log duration: about {parsed['duration_s'] / 60:.1f} minutes ({parsed['duration_s']:.0f} seconds)."})
    intervals = flight_intervals(parsed)
    total_armed = sum(item["duration"] for item in intervals)
    metrics["armed_s"] = total_armed
    notes.append({"level": "info", "text": f"Detected {len(intervals)} complete or open arm interval(s), totaling about {total_armed / 60:.1f} minutes."})
    if "MODE" in parsed["frames"]:
        frame = parsed["frames"]["MODE"]
        col = next((c for c in ("Mode", "ModeNum", "ModeName") if c in frame), None)
        modes = list(dict.fromkeys(frame[col].dropna().astype(str).tolist()))[:12] if col else []
        notes.append({"level": "info", "text": "Flight mode IDs recorded: " + (", ".join(modes) if modes else "mode messages exist, but names were not decoded") + "."})

    gps_sats = stat_value(parsed, "GPS", "NSats", "minimum")
    gps_status = stat_value(parsed, "GPS", "Status", "minimum")
    gps_status_max = stat_value(parsed, "GPS", "Status", "maximum")
    if gps_sats is not None:
        metrics["gps_min_sats"] = gps_sats
        avg_sats = stat_value(parsed, "GPS", "NSats")
        notes.append({"level": "info", "text": f"GPS satellites: minimum {gps_sats:.0f}, average {_number(avg_sats, 1)}."})
        if gps_sats < 6:
            notes.append({"level": "warning", "text": "The logged satellite count dropped below 6 at least once. Inspect the time plot and GPS quality before relying on position-based flight modes."})
    if gps_status is not None and gps_status < 3:
        notes.append({"level": "warning", "text": "GPS reports a fix below 3D at some point in the log."})

    volts_min = stat_value(parsed, "BAT", "Volt", "minimum")
    volts_max = stat_value(parsed, "BAT", "Volt", "maximum")
    volts_first = stat_value(parsed, "BAT", "Volt", "first")
    volts_last = stat_value(parsed, "BAT", "Volt", "last")
    if volts_min is not None:
        metrics.update({"battery_min_v": volts_min, "battery_max_v": volts_max})
        notes.append({"level": "info", "text": f"Battery voltage range: {volts_min:.2f} to {volts_max:.2f} V. Compare this with the battery cell count, chemistry, and configured failsafe thresholds."})
        if volts_first and volts_last is not None and volts_last < volts_first * 0.85:
            notes.append({"level": "warning", "text": "The final logged voltage is more than 15% below the starting value. Review battery sizing and voltage sag under load."})
    peak_current = stat_value(parsed, "BAT", "Curr", "maximum")
    if peak_current is not None:
        metrics["battery_peak_a"] = peak_current
        notes.append({"level": "info", "text": f"Peak recorded current: {peak_current:.1f} A. Compare it with the ratings for the battery and power system."})
    peak_vibe = max((stat_value(parsed, "VIBE", col, "maximum") for col in ("VibeX", "VibeY", "VibeZ") if stat_value(parsed, "VIBE", col, "maximum") is not None), default=None)
    if peak_vibe is not None:
        metrics["vibration_peak"] = peak_vibe
        notes.append({"level": "info", "text": f"Peak vibration sample: {peak_vibe:.1f} logged units. Review the graph for sustained high values or clipping; a single peak is not a diagnosis."})
        if peak_vibe > 30:
            notes.append({"level": "warning", "text": "At least one vibration axis exceeded 30. Inspect vibration and clipping messages, and check for mechanical imbalance or loose mounting."})
    err_count = parsed["counts"].get("ERR", 0)
    notes.append({"level": "warning" if err_count else "info", "text": f"Recorded ERR messages: {err_count}. Match each error code to the firmware version before diagnosing it." if err_count else "No ERR message records were found. This does not guarantee the flight had no problems."})
    if "ATT" in parsed["frames"]:
        notes.append({"level": "info", "text": "Attitude measurements are available. The PDF includes roll, pitch, yaw, and attitude error plots when recorded."})
    if "GPS" in parsed["frames"]:
        notes.append({"level": "info", "text": "The log contains GPS coordinates. They are shown only in the report and are not sent to a geocoding or weather service."})
    notes.append({"level": "info", "text": "Ambient humidity, wind, and sky conditions are not present in this DataFlash log; the PDF marks them as unavailable rather than looking them up online."})
    return notes, metrics


def _safe_float(series: pd.Series) -> pd.Series:
    return pd.to_numeric(series, errors="coerce")


def _unit_for(message: str, field: str) -> str:
    return {
        ("BAT", "Volt"): "V", ("BAT", "Curr"): "A", ("BAT", "Power"): "W",
        ("BAT", "CurrTot"): "mAh", ("BAT", "EnrgTot"): "Wh", ("BARO", "Alt"): "m",
        ("BARO", "Temp"): "deg C", ("GPS", "NSats"): "satellites", ("GPS", "HDop"): "HDOP",
        ("ATT", "ErrRP"): "error", ("ATT", "ErrYaw"): "error", ("VIBE", "VibeZ"): "vibration",
    }.get((message, field), "")


def _series_stats(parsed: dict[str, Any], message: str, field: str) -> str:
    stat = parsed["stats"].get(message, {}).get(field)
    if stat is None or not stat.count:
        return ""
    suffix = _unit_for(message, field)
    return f"{field}: avg {_number(stat.average, 2, ' ' + suffix if suffix else '')}, min {_number(stat.minimum, 2, ' ' + suffix if suffix else '')}, max {_number(stat.maximum, 2, ' ' + suffix if suffix else '')}"


def _chart_png(parsed: dict[str, Any], title: str, series: list[tuple[str, str, str]], dual_axis: bool = False) -> bytes | None:
    """Create a single plot image from bounded samples, with logged arm/land event markers."""
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    active = []
    for message, field, label in series:
        frame = parsed["frames"].get(message)
        if frame is not None and field in frame and "Elapsed (s)" in frame:
            active.append((message, field, label, frame))
    if not active:
        return None
    fig = Figure(figsize=(11.5, 6.3), dpi=140, facecolor="white")
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    right = ax.twinx() if dual_axis and len(active) > 1 else None
    colors = ["#1477b8", "#e53935", "#2e8b57", "#ff8c00", "#7b61a8", "#555555"]
    summary = []
    for index, (message, field, label, frame) in enumerate(active):
        x = _safe_float(frame["Elapsed (s)"])
        y = _safe_float(frame[field])
        valid = x.notna() & y.notna()
        if not valid.any():
            continue
        axis = right if right is not None and index == 1 else ax
        color = colors[index % len(colors)]
        axis.plot(x[valid], y[valid], color=color, linewidth=1.15, label=label)
        axis.set_ylabel(label, color=color, fontsize=9)
        axis.tick_params(axis="y", labelcolor=color)
        message_kind = "BAT" if field in {"Volt", "Curr", "CurrTot", "EnrgTot", "Power"} else message
        summary.append(_series_stats(parsed, message_kind, field))
    ax.set_title(title + ("\n" + "  |  ".join(x for x in summary if x) if summary else ""), fontsize=12, pad=12)
    ax.set_xlabel("Time since log start (s)")
    ax.grid(True, color="#b0b0b0", linewidth=0.65, alpha=0.8)
    ax.set_axisbelow(True)
    if right is not None:
        handles, labels = ax.get_legend_handles_labels()
        handles2, labels2 = right.get_legend_handles_labels()
        ax.legend(handles + handles2, labels + labels2, loc="upper left", fontsize=8, framealpha=0.9)
    else:
        ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    for event in parsed["events"]:
        name = event.get("Event")
        t = event.get("Elapsed (s)")
        if name and t is not None and name in {"ARMED", "DISARMED", "LAND_COMPLETE", "LOST_GPS"}:
            ax.axvline(float(t), color="#c62828", linestyle=":", linewidth=0.8, alpha=0.75)
    fig.tight_layout()
    output = BytesIO()
    fig.savefig(output, format="png", dpi=140, bbox_inches="tight")
    fig.clear()
    return output.getvalue()


def _gps_frame(parsed: dict[str, Any], instance: int) -> pd.DataFrame | None:
    frame = parsed["frames"].get("GPS")
    if frame is None or "I" not in frame:
        return frame if instance == 0 else None
    gps_i = pd.to_numeric(frame["I"], errors="coerce")
    result = frame[gps_i == instance]
    return result if not result.empty else None


def _make_chart_specs(parsed: dict[str, Any]) -> list[tuple[str, list[tuple[str, str, str]], bool]]:
    specs: list[tuple[str, list[tuple[str, str, str]], bool]] = []
    frames = parsed["frames"]
    if "BAT" in frames and "BARO" in frames:
        for field, label in (("Volt", "Battery voltage (V)"), ("Curr", "Battery current (A)")):
            if field in frames["BAT"]:
                specs.append((f"BAT_{field} & BARO_Alt vs Time", [("BAT", field, label), ("BARO", "Alt", "Altitude (m)")], True))
        bat = frames["BAT"].copy()
        if {"Volt", "Curr"}.issubset(bat.columns):
            bat["Power"] = _safe_float(bat["Volt"]) * _safe_float(bat["Curr"])
            frames["BAT"] = bat
            specs.append(("BAT_Power & BARO_Alt vs Time", [("BAT", "Power", "Power (W)"), ("BARO", "Alt", "Altitude (m)")], True))
        if {"CurrTot", "EnrgTot"}.issubset(frames["BAT"].columns):
            specs.append(("Battery CurrTot vs EnrgTot", [("BAT", "CurrTot", "Consumed charge (mAh)"), ("BAT", "EnrgTot", "Consumed energy (Wh)")], True))
    if "RCOU" in frames:
        available = [f"C{i}" for i in range(1, 5) if f"C{i}" in frames["RCOU"]]
        if available:
            specs.append(("RCOU Channels vs Time", [("RCOU", c, f"Output {c[1:]}") for c in available], False))
    if "VIBE" in frames:
        for axis in ("VibeZ", "VibeX", "VibeY"):
            if axis in frames["VIBE"]:
                specs.append((f"VIBE_{axis} vs Time", [("VIBE", axis, f"Vibration {axis[-1]}")], False))
                break
    for instance in (0, 1):
        gps = _gps_frame(parsed, instance)
        if gps is not None and {"NSats", "HDop"}.issubset(gps.columns):
            frames[f"GPS{instance}"] = gps
            specs.append((f"GPS{instance} NSats & HDop vs Time", [(f"GPS{instance}", "NSats", "Satellite count"), (f"GPS{instance}", "HDop", "HDOP")], True))
    if "MAG" in frames and "AHR2" in frames:
        mag = frames["MAG"]
        mag_fields = [f for f in ("Res", "MagX", "MagY", "MagZ") if f in mag]
        if mag_fields and "Yaw" in frames["AHR2"]:
            specs.append(("MAG field & AHR2_Yaw vs Time", [("MAG", mag_fields[0], f"Compass {mag_fields[0]}"), ("AHR2", "Yaw", "AHR2 yaw (deg)")], True))
    ekf_message = next((name for name in ("XKF4", "NKF4") if name in frames and any(c in frames[name] for c in ("SM", "SV", "SP"))), None)
    if "MAG" in frames and ekf_message:
        mag_field = next((c for c in ("Res", "MagX", "MagY", "MagZ") if c in frames["MAG"]), None)
        ekf_field = next((c for c in ("SM", "SV", "SP") if c in frames[ekf_message]), None)
        if mag_field and ekf_field:
            specs.append((f"MAG field & {ekf_message} score vs Time", [("MAG", mag_field, f"Compass {mag_field}"), (ekf_message, ekf_field, f"EKF {ekf_field}")], True))
    if "RCIN" in frames and "AHR2" in frames and "C4" in frames["RCIN"] and "Yaw" in frames["AHR2"]:
        specs.append(("RCIN_C4 and AHR2_Yaw vs Time", [("RCIN", "C4", "RC input C4 (PWM)"), ("AHR2", "Yaw", "AHR2 yaw (deg)")], True))
    if "BARO" in frames and "Alt" in frames["BARO"]:
        if "CTUN" in frames and "DAlt" in frames["CTUN"]:
            specs.append(("BARO_Alt and CTUN_DAlt vs Time", [("BARO", "Alt", "Barometer altitude (m)"), ("CTUN", "DAlt", "Target altitude (m)")], True))
        elif "Temp" in frames["BARO"]:
            specs.append(("BARO_Alt and temperature vs Time", [("BARO", "Alt", "Barometer altitude (m)"), ("BARO", "Temp", "Barometer temperature (deg C)")], True))
    if "ATT" in frames:
        fields = [field for field in ("ErrRP", "ErrYaw") if field in frames["ATT"]]
        if fields:
            specs.append(("ATT Errors vs Time", [("ATT", field, field) for field in fields], False))
    return specs


def _get_location(parsed: dict[str, Any]) -> tuple[str, str]:
    gps = parsed["frames"].get("GPS")
    if gps is None or not {"Lat", "Lng"}.issubset(gps.columns):
        return "Not recorded", "Not recorded"
    data = gps.copy()
    if "Status" in data:
        data = data[pd.to_numeric(data["Status"], errors="coerce") >= 3]
    lat = pd.to_numeric(data["Lat"], errors="coerce").dropna()
    lng = pd.to_numeric(data["Lng"], errors="coerce").dropna()
    if lat.empty or lng.empty:
        return "Not recorded", "Not recorded"
    lat_value, lon_value = float(lat.median()), float(lng.median())
    if abs(lat_value) > 90:
        lat_value /= 1e7
    if abs(lon_value) > 180:
        lon_value /= 1e7
    return f"{lat_value:.6f}", f"{lon_value:.6f}"


def _criteria(parsed: dict[str, Any]) -> list[tuple[str, str]]:
    stats = parsed["stats"]
    result: list[tuple[str, str]] = []
    def health_status(message_names: tuple[str, ...]) -> str:
        vals = []
        for name in message_names:
            s = stats.get(name, {}).get("Health")
            if s and s.count:
                vals.append((s.minimum, s.maximum))
        if not vals:
            return "Not recorded"
        return "Pass" if all(low > 0 and high > 0 for low, high in vals) else "Review"
    result.append(("Accelerometer Health", health_status(("IMU", "IMU2", "IMU3"))))
    result.append(("Gyroscope Health", health_status(("IMU", "IMU2", "IMU3"))))
    result.append(("Barometer Health", health_status(("BARO",))))
    result.append(("Compass Health", health_status(("MAG",))))
    gps_fix = stats.get("GPS", {}).get("Status")
    gps_sats = stats.get("GPS", {}).get("NSats")
    if gps_fix and gps_sats:
        result.append(("GPS Health", "Pass" if gps_fix.minimum >= 3 and gps_sats.minimum >= 6 else "Review"))
    else:
        result.append(("GPS Health", "Not recorded"))
    result.append(("Current consumption", "Review" if stats.get("BAT", {}).get("Curr") else "Not recorded"))
    pwm = stats.get("RCOU", {})
    pwm_stats = [v for k, v in pwm.items() if k.startswith("C") and v.count]
    if pwm_stats:
        result.append(("PWM Output", "Pass" if all(s.minimum >= 800 and s.maximum <= 2200 for s in pwm_stats) else "Review"))
    else:
        result.append(("PWM Output", "Not recorded"))
    vibe_values = [stats.get("VIBE", {}).get(k) for k in ("VibeX", "VibeY", "VibeZ")]
    vibe_values = [s for s in vibe_values if s and s.count]
    result.append(("Vibration Result", "Pass" if vibe_values and max(s.maximum for s in vibe_values) <= 30 else ("Review" if vibe_values else "Not recorded")))
    result.append(("IOMCU Health", "Review" if parsed["counts"].get("IO", 0) or parsed["counts"].get("IOMCU", 0) else "Not recorded"))
    overall = "Pass" if all(state == "Pass" for _, state in result) else "Review"
    result.append(("Overall Result", overall))
    return result


def build_pdf_report(parsed: dict[str, Any], filename: str, file_size: int) -> bytes:
    """Build the multi-page flight-test PDF style shown in the user's reference."""
    from reportlab.lib import colors
    from reportlab.lib.enums import TA_CENTER, TA_LEFT
    from reportlab.lib.pagesizes import landscape, letter
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch
    from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image

    page_size = landscape(letter)
    output = BytesIO()
    document = SimpleDocTemplate(output, pagesize=page_size, rightMargin=0.4*inch, leftMargin=0.4*inch, topMargin=0.45*inch, bottomMargin=0.48*inch, title="Flight Test Report")
    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle(name="ReportTitle", parent=styles["Title"], fontName="Helvetica-Bold", fontSize=23, leading=28, alignment=TA_CENTER, textColor=colors.HexColor("#17365d"), spaceAfter=8))
    styles.add(ParagraphStyle(name="Section", parent=styles["Heading2"], fontSize=13, leading=16, textColor=colors.HexColor("#24476b"), spaceBefore=8, spaceAfter=5))
    styles.add(ParagraphStyle(name="BodySmall", parent=styles["BodyText"], fontSize=9, leading=12, spaceAfter=4))
    styles.add(ParagraphStyle(name="MonoSmall", parent=styles["Code"], fontName="Courier", fontSize=7.7, leading=10, leftIndent=6, spaceAfter=2))
    styles.add(ParagraphStyle(name="ChartTitle", parent=styles["Heading1"], fontSize=15, leading=18, alignment=TA_CENTER, textColor=colors.HexColor("#17365d"), spaceAfter=5))
    styles.add(ParagraphStyle(name="TableSmall", parent=styles["BodyText"], fontName="Helvetica", fontSize=7.2, leading=8.5, spaceAfter=0))
    styles.add(ParagraphStyle(name="TableLabel", parent=styles["TableSmall"], fontName="Helvetica-Bold"))
    story: list[Any] = []
    filename_safe = escape(Path(filename).name)
    loc_lat, loc_lon = _get_location(parsed)
    duration = float(parsed["duration_s"])
    intervals = flight_intervals(parsed)
    effective = sum(x["duration"] for x in intervals)
    baro_temp = stat_value(parsed, "BARO", "Temp")
    gps_alt = stat_value(parsed, "GPS", "Alt")
    firmware = escape(str(parsed.get("firmware", "Unknown")))
    mb = file_size / (1024 * 1024)

    story.append(Paragraph("Flight Test Report", styles["ReportTitle"]))
    story.append(Paragraph(f"Effective Flight Time: {effective / 60:.2f} minutes", styles["BodySmall"]))
    story.append(Paragraph("=== All Arm &amp; Disarm Timings ===", styles["Section"]))
    if intervals:
        timing_rows = [["Flight", "Arm time (since log start)", "Disarm time (since log start)", "Duration"]]
        for index, item in enumerate(intervals, 1):
            timing_rows.append([f"Flight {index}", _elapsed_text(item["arm"]), _elapsed_text(item["disarm"]), f"{item['duration'] / 60:.2f} min"])
        table = Table(timing_rows, colWidths=[1.0*inch, 2.15*inch, 2.15*inch, 1.1*inch], repeatRows=1)
        table.setStyle(TableStyle([
            ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#4f81bd")), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
            ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"), ("FONTSIZE", (0,0), (-1,-1), 8),
            ("GRID", (0,0), (-1,-1), 0.5, colors.HexColor("#666666")), ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, colors.HexColor("#eef3f8")]),
            ("VALIGN", (0,0), (-1,-1), "MIDDLE"), ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
        ]))
        story.append(table)
    else:
        story.append(Paragraph("No complete arm/disarm interval was decoded from EV messages. Total log span: " + _elapsed_text(duration) + ".", styles["BodySmall"]))

    story.append(Paragraph("=== Location &amp; Weather ===", styles["Section"]))
    raw_info_rows = [
        ["Log file", filename_safe, "Firmware", firmware],
        ["Log size", f"{mb:.1f} MB", "Total log span", _elapsed_text(duration)],
        ["GPS position", f"{loc_lat}, {loc_lon} (decimal degrees)" if loc_lat != "Not recorded" else "Not recorded", "GPS altitude", _number(gps_alt, 2, " m")],
        ["Flight-controller barometer temperature", _number(baro_temp, 1, " deg C"), "Humidity / wind / sky", "Not recorded in this log"],
    ]
    info_rows = [[
        Paragraph(escape(str(row[0])), styles["TableLabel"]),
        Paragraph(escape(str(row[1])), styles["TableSmall"]),
        Paragraph(escape(str(row[2])), styles["TableLabel"]),
        Paragraph(escape(str(row[3])), styles["TableSmall"]),
    ] for row in raw_info_rows]
    info_table = Table(info_rows, colWidths=[1.8*inch, 3.15*inch, 1.55*inch, 3.65*inch])
    info_table.setStyle(TableStyle([
        ("BACKGROUND", (0,0), (0,-1), colors.HexColor("#e8eef5")), ("BACKGROUND", (2,0), (2,-1), colors.HexColor("#e8eef5")),
        ("FONTNAME", (0,0), (0,-1), "Helvetica-Bold"), ("FONTNAME", (2,0), (2,-1), "Helvetica-Bold"),
        ("FONTSIZE", (0,0), (-1,-1), 8), ("GRID", (0,0), (-1,-1), 0.4, colors.HexColor("#8c99a8")),
        ("VALIGN", (0,0), (-1,-1), "MIDDLE"), ("TOPPADDING", (0,0), (-1,-1), 5), ("BOTTOMPADDING", (0,0), (-1,-1), 5),
    ]))
    story.append(info_table)
    story.append(Spacer(1, 8))
    story.append(Paragraph("Weather conditions are not recorded in the DataFlash log. No outside weather or reverse-geocoding service was queried. GPS coordinates are included for the owner's review.", styles["BodySmall"]))
    story.append(Paragraph("This automated report is a screening aid, not a flight-safety certification. Interpret the plots against the vehicle configuration and the firmware used for this flight.", styles["BodySmall"]))

    for title, series, dual in _make_chart_specs(parsed):
        png = _chart_png(parsed, title, series, dual_axis=dual)
        if png is None:
            continue
        story.append(PageBreak())
        story.append(Paragraph(escape(title), styles["ChartTitle"]))
        story.append(Image(BytesIO(png), width=10.2*inch, height=6.1*inch, kind="proportional"))

    story.append(PageBreak())
    story.append(Paragraph("ArduPilot Messages", styles["Section"]))
    all_messages = [e for e in parsed["events"] if e.get("_type") in {"MSG", "ERR", "EV", "MODE"}]
    errors = [e for e in all_messages if e.get("_type") == "ERR"][:12]
    milestones = [e for e in all_messages if e.get("_type") == "EV" and e.get("Event") in {"ARMED", "DISARMED", "LAND_COMPLETE", "LOST_GPS"}][:13]
    priority_messages = list({id(e): e for e in errors + milestones}.values())[:25]
    startup = [e for e in all_messages if e.get("_type") in {"MSG", "MODE"}]
    selected = {id(e): e for e in priority_messages}
    for item in startup:
        if len(selected) >= 25:
            break
        selected[id(item)] = item
    messages = sorted(selected.values(), key=lambda e: float(e.get("Elapsed (s)") or 0))
    if not messages:
        story.append(Paragraph("No MSG, ERR, EV, or MODE entries were found.", styles["BodySmall"]))
    else:
        story.append(Paragraph(f"Showing {len(messages)} selected startup, mode, arm/disarm, and error records from {len(all_messages)} total. Entries are in log order; recognized EV values use standard ArduPilot event IDs.", styles["BodySmall"]))
        message_data: list[list[Any]] = [["Time (s)", "Type", "Message / event"]]
        for item in messages[:1200]:
            kind = item.get("_type", "")
            if kind == "MSG":
                text = item.get("Message", item.get("Msg", ""))
            elif kind == "EV":
                text = item.get("Event", f"Event {item.get('Id', '')}")
            elif kind == "MODE":
                text = "Mode " + str(item.get("Mode", item.get("ModeNum", item.get("ModeName", ""))))
            else:
                text = f"Subsys {item.get('Subsys', '?')}, code {item.get('ECode', '?')}"
            message_data.append([_number(item.get("Elapsed (s)"), 2), kind, escape(str(text))])
        msg_table = Table(message_data, colWidths=[1.0*inch, 0.65*inch, 8.75*inch], repeatRows=1, splitByRow=1)
        msg_style = [
            ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#4f81bd")), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
            ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"), ("FONTSIZE", (0,0), (-1,-1), 7.3),
            ("GRID", (0,0), (-1,-1), 0.35, colors.HexColor("#a0a0a0")), ("VALIGN", (0,0), (-1,-1), "TOP"),
            ("ROWBACKGROUNDS", (0,1), (-1,-1), [colors.white, colors.HexColor("#f2f5f8")]),
        ]
        for row_index, item in enumerate(messages[:1200], 1):
            if item.get("_type") == "ERR":
                msg_style.append(("TEXTCOLOR", (0,row_index), (-1,row_index), colors.HexColor("#c00000")))
        msg_table.setStyle(TableStyle(msg_style))
        story.append(msg_table)

    story.append(PageBreak())
    story.append(Paragraph("Flight Pass/Fail Criteria Report", styles["ReportTitle"]))
    criteria = _criteria(parsed)
    rows = [["Criteria", "Result"]] + [[name, status] for name, status in criteria]
    criteria_table = Table(rows, colWidths=[3.0*inch, 3.0*inch], repeatRows=1, hAlign="CENTER")
    styles_by_status = {"Pass": colors.HexColor("#e2f0d9"), "Review": colors.HexColor("#fff2cc"), "Not recorded": colors.HexColor("#e7e6e6")}
    table_style = [
        ("BACKGROUND", (0,0), (-1,0), colors.HexColor("#4f81bd")), ("TEXTCOLOR", (0,0), (-1,0), colors.white),
        ("FONTNAME", (0,0), (-1,0), "Helvetica-Bold"), ("FONTSIZE", (0,0), (-1,-1), 10),
        ("GRID", (0,0), (-1,-1), 0.65, colors.black), ("ALIGN", (0,0), (-1,-1), "CENTER"),
        ("VALIGN", (0,0), (-1,-1), "MIDDLE"), ("TOPPADDING", (0,0), (-1,-1), 8), ("BOTTOMPADDING", (0,0), (-1,-1), 8),
    ]
    for index, (_, status) in enumerate(criteria, 1):
        table_style.append(("BACKGROUND", (1,index), (1,index), styles_by_status.get(status, colors.white)))
        table_style.append(("TEXTCOLOR", (1,index), (1,index), colors.HexColor("#267326") if status == "Pass" else colors.HexColor("#9c6500") if status == "Review" else colors.HexColor("#555555")))
    criteria_table.setStyle(TableStyle(table_style))
    story.append(Spacer(1, 16))
    story.append(criteria_table)
    story.append(Spacer(1, 12))
    story.append(Paragraph("Pass means only that the recorded data met this report's basic screening rule. Review means manual review is needed. Not recorded means the log did not contain the fields required for that check. Current consumption is marked Review because a battery capacity and cell configuration are required for a meaningful limit.", styles["BodySmall"]))

    def footer(canvas: Any, doc: Any) -> None:
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#c9d2dc"))
        canvas.line(0.4*inch, 0.34*inch, page_size[0] - 0.4*inch, 0.34*inch)
        canvas.setFont("Helvetica", 7)
        canvas.setFillColor(colors.HexColor("#64748b"))
        canvas.drawString(0.4*inch, 0.2*inch, "ArduPilot Flight Test Report - automated screening")
        canvas.drawRightString(page_size[0] - 0.4*inch, 0.2*inch, f"Page {doc.page}")
        canvas.restoreState()

    document.build(story, onFirstPage=footer, onLaterPages=footer)
    return output.getvalue()

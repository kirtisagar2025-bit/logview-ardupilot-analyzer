from __future__ import annotations

import os
from pathlib import Path

import pandas as pd
import plotly.express as px
import streamlit as st

from log_report import build_pdf_report, copy_upload_to_temp, get_summary, parse_bin

st.set_page_config(page_title="Logview | ArduPilot Log Analyzer", page_icon="🚁", layout="wide")
st.title("🚁 Logview - ArduPilot Flight Log Analyzer")
st.write("Upload a DataFlash **.bin** log for beginner-friendly findings, interactive plots, and a detailed PDF flight-test report.")
st.info("This is an automated screening aid, not a safety certification. Verify important findings against the aircraft, its configuration, and the flight context.")

UPLOAD_LIMIT_MB = 150
REPORT_VERSION = 4
WORK_ROOT = Path(__file__).resolve().parent / "work"
TEMP_DIR = WORK_ROOT / "uploaded-logs"

uploaded = st.file_uploader(
    "Choose an ArduPilot DataFlash log",
    type=["bin"],
    max_upload_size=UPLOAD_LIMIT_MB,
    help="Maximum file size on the free hosted version: 150 MB.",
)

if uploaded is None:
    st.markdown("### Getting started\nChoose a `.bin` log below 150 MB. The hosted app analyzes the DataFlash log and provides a PDF with a summary, timing table, plots, message list, and screening criteria.")
    st.caption("Weather such as wind, humidity, and sky conditions is not normally recorded in DataFlash logs. The report labels unavailable values instead of looking them up using GPS coordinates.")
else:
    upload_id = getattr(uploaded, "file_id", None) or f"{uploaded.name}:{uploaded.size}"
    if st.session_state.get("analysis_upload_id") != upload_id:
        staged_path = None
        try:
            with st.spinner(f"Staging {uploaded.size / (1024 * 1024):,.1f} MB log in chunks…"):
                staged_path = copy_upload_to_temp(uploaded, TEMP_DIR)
            with st.spinner("Reading the DataFlash log and building bounded plot samples…"):
                parsed = parse_bin(staged_path)
            os.unlink(staged_path)  # parsed reader has closed; Windows can now release the temp file
            staged_path = None
            with st.spinner("Creating the multi-page PDF report…"):
                notes, metrics = get_summary(parsed, uploaded.name)
                pdf_bytes = build_pdf_report(parsed, uploaded.name, uploaded.size)
            st.session_state["analysis_upload_id"] = upload_id
            st.session_state["analysis_parsed"] = parsed
            st.session_state["analysis_notes"] = notes
            st.session_state["analysis_metrics"] = metrics
            st.session_state["analysis_pdf"] = pdf_bytes
            st.session_state["analysis_filename"] = Path(uploaded.name).name
            st.session_state["analysis_report_version"] = REPORT_VERSION
        except Exception as exc:
            st.session_state.pop("analysis_upload_id", None)
            st.error(f"Could not analyze this file: {exc}")
            st.caption("The temporary file is removed after the reader closes. Check that this is a complete ArduPilot DataFlash .bin log and that the computer has enough free disk space and memory.")
        finally:
            if staged_path:
                try:
                    os.unlink(staged_path)
                except FileNotFoundError:
                    pass

    elif st.session_state.get("analysis_report_version") != REPORT_VERSION:
        with st.spinner("Refreshing the report layout from the parsed log…"):
            parsed = st.session_state["analysis_parsed"]
            filename = st.session_state["analysis_filename"]
            notes, metrics = get_summary(parsed, filename)
            pdf_bytes = build_pdf_report(parsed, filename, uploaded.size)
            st.session_state["analysis_notes"] = notes
            st.session_state["analysis_metrics"] = metrics
            st.session_state["analysis_pdf"] = pdf_bytes
            st.session_state["analysis_report_version"] = REPORT_VERSION

    if st.session_state.get("analysis_upload_id") == upload_id:
        parsed = st.session_state["analysis_parsed"]
        notes = st.session_state["analysis_notes"]
        metrics = st.session_state["analysis_metrics"]
        filename = st.session_state["analysis_filename"]
        stem = Path(filename).stem

        st.success(f"Analyzed {filename} · {parsed['firmware']} · {len(parsed['counts'])} message types")
        c1, c2, c3, c4 = st.columns(4)
        c1.metric("Log duration", f"{metrics.get('duration_s', 0) / 60:.1f} min")
        c2.metric("Armed time", f"{metrics.get('armed_s', 0) / 60:.1f} min")
        c3.metric("Message types", len(parsed["counts"]))
        c4.metric("ERR records", parsed["counts"].get("ERR", 0))

        st.download_button(
            "Download Flight Test Report (PDF)",
            data=st.session_state["analysis_pdf"],
            file_name=f"{stem}_Report.pdf",
            mime="application/pdf",
            type="primary",
        )

        st.subheader("Beginner summary")
        for note in notes:
            if note["level"] == "warning":
                st.warning(note["text"])
            else:
                st.markdown(f"- {note['text']}")

        st.subheader("Flight plots")
        choices = {
            "Attitude (ATT)": ("ATT", ["Roll", "Pitch", "Yaw"]),
            "Battery (BAT)": ("BAT", ["Volt", "Curr", "CurrTot"]),
            "GPS quality (GPS)": ("GPS", ["NSats", "HDop", "Status", "Spd"]),
            "Altitude and throttle (CTUN)": ("CTUN", ["Alt", "DAlt", "ThO", "ThH"]),
            "Vibration (VIBE)": ("VIBE", ["VibeX", "VibeY", "VibeZ", "Clip"]),
            "RC outputs (RCOU)": ("RCOU", [f"C{i}" for i in range(1, 9)]),
            "Position (POS)": ("POS", ["Lat", "Lng", "Alt"]),
        }
        available = [label for label, (kind, fields) in choices.items() if kind in parsed["frames"] and any(f in parsed["frames"][kind] for f in fields)]
        if available:
            selected = st.selectbox("Choose a plot", available)
            kind, fields = choices[selected]
            frame = parsed["frames"][kind]
            columns = [field for field in fields if field in frame]
            data = frame[["Elapsed (s)"] + columns].copy()
            long = data.melt(id_vars="Elapsed (s)", var_name="Signal", value_name="Value")
            long["Value"] = pd.to_numeric(long["Value"], errors="coerce")
            fig = px.line(long, x="Elapsed (s)", y="Value", color="Signal", title=selected, render_mode="webgl")
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.warning("No common plotting messages were found. Check the inventory and the downloadable PDF for available signals.")

        if parsed["events"]:
            st.subheader("Recorded messages, events, and errors")
            rows = []
            visible_events = parsed["events"][:500]
            for item in visible_events:
                kind = item.get("_type", "")
                if kind == "MSG":
                    text = item.get("Message", item.get("Msg", ""))
                elif kind == "EV":
                    text = item.get("Event", f"Event {item.get('Id', '')}")
                elif kind == "ERR":
                    text = f"Subsys {item.get('Subsys', '?')}, code {item.get('ECode', '?')}"
                else:
                    text = f"Mode {item.get('Mode', item.get('ModeNum', item.get('ModeName', '')))}"
                rows.append({"Time (s)": item.get("Elapsed (s)"), "Type": kind, "Message": text})
            st.dataframe(pd.DataFrame(rows), use_container_width=True, hide_index=True)
            if len(parsed["events"]) > len(visible_events):
                st.caption(f"Showing the first {len(visible_events)} of {len(parsed['events'])} event and message records. The PDF selects startup, mode, arm/disarm, and error records for its compact message page.")

        with st.expander("Message inventory"):
            st.dataframe(pd.DataFrame(parsed["inventory"]), use_container_width=True, hide_index=True)

st.caption("This free hosted version accepts logs up to 150 MB. Uploaded logs are processed by the hosting service. Do not upload flight logs unless you are comfortable sending them to the host.")

"""Tests for ReportData (0x700) signal ID compliance.

Verifies that synthetic broadcast uses only Signal IDs listed in
sygnal_docs v2.1.1 mcm-can-api-guide.md § Report.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from mcm_simulator.simulator import McmSimulator, SystemState, _load_dbc
from mcm_simulator.device_config import DeviceSpec
from mcm_simulator.simulator import _DeviceNode, SubsystemState

_REPORT_DATA_ID = 1792  # 0x700

# Documented signal IDs from mcm-can-api-guide.md §Report
_DOCUMENTED_SIGNAL_IDS = {
    0x80,  # Signal Input
    0x81,  # Signal Output Actual
    0x83,  # Signal Output Target
    0x50,  # Feedback Raw
    0x52,  # Closed Loop Setpoint
    0x40,  # Override State
}

# Wheel-speed IDs (0x70–0x74) that were previously broadcast but are NOT in docs
_UNDOCUMENTED_SIGNAL_IDS = {0x70, 0x71, 0x72, 0x73, 0x74}


def _make_sim_for_report() -> McmSimulator:
    db = _load_dbc()
    args = MagicMock()
    args.bus_address = 1
    args.watchdog_timeout_ms = 200
    args.heartbeat_rate_hz = 15.0
    args.report_rate_hz = 10.0
    args.subsystem_ids = [0]
    args.devices = None
    args.socket_path = "/tmp/test.sock"

    sim = McmSimulator.__new__(McmSimulator)
    sim._db = db
    sim._bus = MagicMock()
    sim._args = args

    sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=0.2)
    sub.state = SystemState.HUMAN_CONTROL
    spec = DeviceSpec(bus_address=1, product_type="mcm", product_id=1)
    node = _DeviceNode(spec=spec, subsystems=[sub])
    sim._device_nodes = [node]
    sim._subsystems = [sub]

    return sim


class TestReportSignalIds:
    def test_only_documented_signal_ids_are_broadcast(self) -> None:
        sim = _make_sim_for_report()
        db = sim._db
        # Simulate one report tick (tick=1, interface_id=0)
        for signal_id, _ in sim._REPORT_SIGNALS:
            sim._publish_report_signal(
                bus_address=1, subsystem_id=0, interface_id=0,
                signal_id=signal_id, value=0.0
            )
        sent_ids = set()
        for call in sim._bus.send.call_args_list:
            msg = call.args[0]
            if msg.arbitration_id == _REPORT_DATA_ID:
                decoded = db.decode_message(_REPORT_DATA_ID, msg.data, decode_choices=False)
                sent_ids.add(int(decoded["Report_SignalID"]))
        assert sent_ids == _DOCUMENTED_SIGNAL_IDS, (
            f"Unexpected signal IDs: {sent_ids - _DOCUMENTED_SIGNAL_IDS}, "
            f"Missing: {_DOCUMENTED_SIGNAL_IDS - sent_ids}"
        )

    def test_no_undocumented_wheel_speed_ids_broadcast(self) -> None:
        sim = _make_sim_for_report()
        db = sim._db
        for signal_id, _ in sim._REPORT_SIGNALS:
            sim._publish_report_signal(
                bus_address=1, subsystem_id=0, interface_id=0,
                signal_id=signal_id, value=0.0
            )
        sent_ids = set()
        for call in sim._bus.send.call_args_list:
            msg = call.args[0]
            if msg.arbitration_id == _REPORT_DATA_ID:
                decoded = db.decode_message(_REPORT_DATA_ID, msg.data, decode_choices=False)
                sent_ids.add(int(decoded["Report_SignalID"]))
        overlap = sent_ids & _UNDOCUMENTED_SIGNAL_IDS
        assert overlap == set(), f"Undocumented wheel-speed IDs in broadcast: {overlap}"

    def test_each_frame_round_trips_through_dbc(self) -> None:
        sim = _make_sim_for_report()
        db = sim._db
        for signal_id, _ in sim._REPORT_SIGNALS:
            value = sim._synthetic_value(signal_id, tick=1)
            sim._publish_report_signal(
                bus_address=1, subsystem_id=0, interface_id=0,
                signal_id=signal_id, value=value
            )
        report_frames = [
            call.args[0]
            for call in sim._bus.send.call_args_list
            if call.args[0].arbitration_id == _REPORT_DATA_ID
        ]
        assert len(report_frames) == len(sim._REPORT_SIGNALS)
        for frame in report_frames:
            decoded = db.decode_message(_REPORT_DATA_ID, frame.data)
            assert "Report_SignalID" in decoded
            assert "Report_Value_FLOAT32" in decoded

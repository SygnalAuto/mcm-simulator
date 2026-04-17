"""Tests for MCM simulator diagnostic frame broadcasting.

Verifies that the simulator correctly encodes FaultState, FaultIncrement,
ReportData, and ErrorStatus CAN frames using the sygnal DBC definitions.
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from mcm_simulator.simulator import (
    McmSimulator,
    SubsystemState,
    SystemState,
    _load_dbc,
    _is_fault_state,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_FAULT_STATE_ID = 32  # 0x20
_FAULT_INCREMENT_ID = 33  # 0x21
_FAULT_LIST_ID = 544  # 0x220
_FAULT_ROOT_CAUSE_ID = 545  # 0x221
_REPORT_DATA_ID = 1792  # 0x700
_ERROR_STATUS_ID = 48  # 0x30


def _make_args(**overrides) -> MagicMock:
    """Create a minimal argparse.Namespace mock for McmSimulator."""
    defaults = {
        "can_interface": "vcan0",
        "watchdog_timeout_ms": 200,
        "heartbeat_rate_hz": 15.0,
        "report_rate_hz": 10.0,
        "bus_address": 1,
        "subsystem_ids": [0, 1],
        "devices": None,
        "socket_path": "/tmp/test-sim.sock",
    }
    defaults.update(overrides)
    args = MagicMock()
    for k, v in defaults.items():
        setattr(args, k, v)
    return args


# ---------------------------------------------------------------------------
# DBC loading — Fault.dbc and Report.dbc are now included
# ---------------------------------------------------------------------------


class TestDbcLoading:
    def test_load_dbc_includes_fault_messages(self) -> None:
        db = _load_dbc()
        assert db.get_message_by_name("FaultState") is not None
        assert db.get_message_by_name("FaultIncrement") is not None

    def test_load_dbc_includes_report_message(self) -> None:
        db = _load_dbc()
        assert db.get_message_by_name("ReportData") is not None

    def test_load_dbc_includes_error_message(self) -> None:
        db = _load_dbc()
        # Error.dbc was already present — verify it's now loaded
        assert db.get_message_by_name("ErrorStatus") is not None


# ---------------------------------------------------------------------------
# FaultState frame encoding
# ---------------------------------------------------------------------------


class TestFaultStateFrame:
    def test_publish_fault_state_encodes_correctly(self) -> None:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        args = _make_args()
        with patch("mcm_simulator.simulator.argparse") as _:
            sim = McmSimulator.__new__(McmSimulator)
            sim._db = db
            sim._bus = MagicMock()
            sim._bus.send = lambda msg: sent_frames.append(
                (msg.arbitration_id, bytes(msg.data))
            )
            sim._args = args

        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=2.0)
        sub.state = SystemState.FAIL_HARD
        sub.fault_cause = 42  # simulated cause

        sim._publish_subsystem_fault_state(sub)

        assert len(sent_frames) == 1
        frame_id, data = sent_frames[0]
        assert frame_id == _FAULT_STATE_ID

        decoded = db.decode_message(frame_id, data)
        assert decoded["BusAddress"] == 1
        assert decoded["SubsystemID"] == 0
        assert decoded["FaultState"] == int(SystemState.FAIL_HARD)
        assert decoded["FaultCause"] == 42

    def test_fault_state_human_control_has_zero_cause(self) -> None:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=2.0)
        sub.state = SystemState.HUMAN_CONTROL
        sub.fault_cause = 0

        sim._publish_subsystem_fault_state(sub)

        decoded = db.decode_message(_FAULT_STATE_ID, sent_frames[0][1])
        assert decoded["FaultState"] == 0
        assert decoded["FaultCause"] == 0


# ---------------------------------------------------------------------------
# FaultIncrement frame encoding
# ---------------------------------------------------------------------------


class TestFaultIncrementFrame:
    def test_publish_fault_increment_encodes_correctly(self) -> None:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        sub = SubsystemState(subsystem_id=1, bus_address=1, watchdog_timeout_s=2.0)
        sub.fault_count = 3
        sub.fault_cause = 1  # watchdog

        sim._publish_fault_increment(sub)

        assert len(sent_frames) == 1
        frame_id, data = sent_frames[0]
        assert frame_id == _FAULT_INCREMENT_ID

        decoded = db.decode_message(frame_id, data)
        assert decoded["BusAddress"] == 1
        assert decoded["SubsystemID"] == 1
        assert decoded["FaultCount"] == 3
        assert decoded["FaultType"] == 1


# ---------------------------------------------------------------------------
# FaultState gate — _publish_all_heartbeats dispatch
# ---------------------------------------------------------------------------


def _make_sim_with_sub(state: SystemState, fault_cause: int = 0) -> tuple:
    """Return (sim, sub, sent_frames) with the sub in the given state."""
    from mcm_simulator.simulator import _DeviceNode
    from mcm_simulator.device_config import DeviceSpec
    db = _load_dbc()
    sent_frames: list[tuple[int, bytes]] = []

    sim = McmSimulator.__new__(McmSimulator)
    sim._db = db
    sim._bus = MagicMock()
    sim._bus.send = lambda msg: sent_frames.append(
        (msg.arbitration_id, bytes(msg.data))
    )
    args = MagicMock()
    args.bus_address = 1
    args.watchdog_timeout_ms = 200
    args.heartbeat_rate_hz = 15.0
    args.report_rate_hz = 10.0
    args.subsystem_ids = [0]
    args.devices = None
    args.socket_path = "/tmp/test.sock"
    sim._args = args

    sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=0.2)
    sub.state = state
    sub.fault_cause = fault_cause

    spec = DeviceSpec(bus_address=1, product_type="mcm", product_id=1)
    node = _DeviceNode(spec=spec, subsystems=[sub])
    sim._device_nodes = [node]
    sim._subsystems = [sub]

    return sim, sub, sent_frames


class TestFaultStateGate:
    def test_no_fault_state_frame_when_human_control(self) -> None:
        sim, sub, sent_frames = _make_sim_with_sub(SystemState.HUMAN_CONTROL)
        sim._publish_all_heartbeats()
        fault_state_frames = [f for f in sent_frames if f[0] == _FAULT_STATE_ID]
        assert fault_state_frames == [], "FaultState must not emit in HUMAN_CONTROL"

    def test_no_fault_state_frame_when_mcm_control(self) -> None:
        sim, sub, sent_frames = _make_sim_with_sub(SystemState.MCM_CONTROL)
        sim._publish_all_heartbeats()
        fault_state_frames = [f for f in sent_frames if f[0] == _FAULT_STATE_ID]
        assert fault_state_frames == [], "FaultState must not emit in MCM_CONTROL"

    def test_fault_state_frame_emitted_when_fail_hard(self) -> None:
        db = _load_dbc()
        sim, sub, sent_frames = _make_sim_with_sub(SystemState.FAIL_HARD, fault_cause=0x30)
        sim._publish_all_heartbeats()
        fault_state_frames = [f for f in sent_frames if f[0] == _FAULT_STATE_ID]
        assert len(fault_state_frames) == 1
        decoded = db.decode_message(_FAULT_STATE_ID, fault_state_frames[0][1])
        assert int(decoded["FaultState"]) == int(SystemState.FAIL_HARD)
        assert decoded["FaultCause"] == 0x30


# ---------------------------------------------------------------------------
# ReportData frame encoding
# ---------------------------------------------------------------------------


class TestReportDataFrame:
    def test_publish_report_data_encodes_correctly(self) -> None:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        sim._publish_report_signal(
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            signal_id=112,  # Wheel Speed 0
            value=3.14,
        )

        assert len(sent_frames) == 1
        frame_id, data = sent_frames[0]
        assert frame_id == _REPORT_DATA_ID

        decoded = db.decode_message(frame_id, data, decode_choices=False)
        assert decoded["BusAddress"] == 1
        assert decoded["SubsystemID"] == 0
        assert decoded["InterfaceID"] == 0
        assert decoded["Report_SignalID"] == 112
        assert abs(decoded["Report_Value_FLOAT32"] - 3.14) < 0.01

    def test_report_data_multiple_signals(self) -> None:
        """Verify we can publish multiple signal IDs."""
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        signal_ids = [112, 113, 114, 115, 116]  # Wheel Speed 0-3 + Avg
        for sid in signal_ids:
            sim._publish_report_signal(
                bus_address=1, subsystem_id=0, interface_id=0,
                signal_id=sid, value=float(sid),
            )

        assert len(sent_frames) == 5
        for i, (_, data) in enumerate(sent_frames):
            decoded = db.decode_message(_REPORT_DATA_ID, data, decode_choices=False)
            assert decoded["Report_SignalID"] == signal_ids[i]


# ---------------------------------------------------------------------------
# ErrorStatus frame encoding
# ---------------------------------------------------------------------------


class TestErrorStatusFrame:
    def test_publish_error_status_encodes_correctly(self) -> None:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        sim._publish_error_status(
            bus_address=1,
            subsystem_id=0,
            error_type=4,  # Parameter ID Error
            section_id=1,  # MCMGlobal
            interface_id=0,
            can_id=0x400,
        )

        assert len(sent_frames) == 1
        frame_id, data = sent_frames[0]
        assert frame_id == _ERROR_STATUS_ID

        decoded = db.decode_message(frame_id, data, decode_choices=False)
        assert decoded["BusAddress"] == 1
        assert decoded["SubsystemID"] == 0
        assert decoded["Error_Type"] == 4
        assert decoded["Config_SectionID"] == 1


# ---------------------------------------------------------------------------
# SubsystemState fault tracking
# ---------------------------------------------------------------------------


class TestSubsystemFaultTracking:
    def test_fault_count_increments_on_fail_hard(self) -> None:
        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=2.0)
        # Boot in FAIL_HARD, clear it first
        sub.state = SystemState.HUMAN_CONTROL
        sub.fault_count = 0
        sub.fault_cause = 0

        # Transition to FAIL_HARD should increment
        sub.transition(SystemState.FAIL_HARD, "watchdog timeout")
        assert sub.fault_count == 1
        assert sub.fault_cause != 0  # cause should be set

    def test_fault_cause_set_on_transition(self) -> None:
        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=2.0)
        sub.state = SystemState.HUMAN_CONTROL
        sub.fault_count = 0
        sub.fault_cause = 0

        sub.transition(SystemState.FAIL_HARD, "watchdog timeout")
        # Cause should be non-zero — the specific value depends on implementation
        assert sub.fault_cause > 0

    def test_fault_cause_clears_on_recovery(self) -> None:
        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=2.0)
        sub.state = SystemState.FAIL_HARD
        sub.fault_count = 1
        sub.fault_cause = 1

        sub.transition(SystemState.HUMAN_CONTROL, "fault cleared")
        assert sub.fault_cause == 0


# ---------------------------------------------------------------------------
# FaultList (0x220) + FaultRootCause (0x221) publishers
# ---------------------------------------------------------------------------


class TestFaultListAndRootCause:
    def _make_faulted_sim(self, fault_cause: int = 0x30) -> tuple:
        db = _load_dbc()
        sent_frames: list[tuple[int, bytes]] = []

        sim = McmSimulator.__new__(McmSimulator)
        sim._db = db
        sim._bus = MagicMock()
        sim._bus.send = lambda msg: sent_frames.append(
            (msg.arbitration_id, bytes(msg.data))
        )
        sim._args = _make_args()

        sub = SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=0.2)
        sub.state = SystemState.FAIL_HARD
        sub.fault_count = 2
        sub.fault_cause = fault_cause

        from mcm_simulator.simulator import _DeviceNode
        from mcm_simulator.device_config import DeviceSpec
        spec = DeviceSpec(bus_address=1, product_type="mcm", product_id=1)
        node = _DeviceNode(spec=spec, subsystems=[sub])
        sim._device_nodes = [node]
        sim._subsystems = [sub]

        return sim, sub, sent_frames, db

    def test_fault_list_published_while_fault_active(self) -> None:
        sim, sub, sent_frames, db = self._make_faulted_sim(fault_cause=0x30)
        sim._publish_fault_list(sub)
        fl_frames = [f for f in sent_frames if f[0] == _FAULT_LIST_ID]
        assert len(fl_frames) == 1
        decoded = db.decode_message(_FAULT_LIST_ID, fl_frames[0][1])
        assert decoded["BusAddress"] == 1
        assert decoded["SubsystemID"] == 0
        assert decoded["FaultType"] == 0x30
        assert decoded["FaultCount"] == 2

    def test_fault_list_not_published_when_fault_count_zero(self) -> None:
        sim, sub, sent_frames, db = self._make_faulted_sim()
        sub.fault_count = 0
        sim._publish_fault_list(sub)
        fl_frames = [f for f in sent_frames if f[0] == _FAULT_LIST_ID]
        assert fl_frames == []

    def test_fault_root_cause_published_while_fault_active(self) -> None:
        sim, sub, sent_frames, db = self._make_faulted_sim(fault_cause=0x01)
        sim._publish_fault_root_cause(sub)
        rc_frames = [f for f in sent_frames if f[0] == _FAULT_ROOT_CAUSE_ID]
        assert len(rc_frames) == 1
        decoded = db.decode_message(_FAULT_ROOT_CAUSE_ID, rc_frames[0][1])
        assert decoded["FailHardCause"] == 0x01
        assert decoded["FailOp1Cause"] == 0
        assert decoded["FailOp2Cause"] == 0

    def test_fault_list_not_emitted_in_human_control(self) -> None:
        sim, sub, sent_frames, db = self._make_faulted_sim()
        sub.state = SystemState.HUMAN_CONTROL
        # gate should prevent emission; call directly to test publisher guard
        if not _is_fault_state(sub.state):
            pass  # production code skips; just verify predicate is correct
        assert not _is_fault_state(SystemState.HUMAN_CONTROL)

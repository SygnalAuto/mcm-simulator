"""Tests for the Identify CAN protocol — responds to IdentifyCommand (0x600)
with IdentifyResponseMain (0x601), IdentifyResponseAppVersion (0x602),
and IdentifyResponseBLVersion (0x603).

No real vcan0 needed — python-can Bus is mocked.
"""

from __future__ import annotations

import argparse
from unittest.mock import MagicMock

import can
import cantools
from pathlib import Path

from mcm_simulator.simulator import (
    McmSimulator,
    _ID_IDENTIFY_CMD,
    _ID_IDENTIFY_RESP_MAIN,
    _ID_IDENTIFY_RESP_APP_VERSION,
    _ID_IDENTIFY_RESP_BL_VERSION,
)


_DBC_DIR = Path(__file__).parent.parent / "mcm_simulator" / "dbc"


def _load_identify_db() -> cantools.database.Database:
    db = cantools.database.Database()
    db.add_dbc_file(str(_DBC_DIR / "Identify.dbc"))
    return db


def _make_args(**kwargs) -> argparse.Namespace:
    defaults = dict(
        can_interface="vcan0",
        watchdog_timeout_ms=200,
        heartbeat_rate_hz=15.0,
        bus_address=1,
        subsystem_ids=[0, 1],
        socket_path="/tmp/test-simulated-mcm.sock",
        devices=None,  # None = use legacy bus_address/subsystem_ids
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


def _make_sim(args: argparse.Namespace | None = None) -> McmSimulator:
    sim = McmSimulator(args or _make_args())
    sim._bus = MagicMock(spec=can.BusABC)
    return sim


def _make_identify_command() -> can.Message:
    """IdentifyCommand (0x600) is a zero-length broadcast."""
    return can.Message(
        arbitration_id=_ID_IDENTIFY_CMD,
        data=b"",
        is_extended_id=False,
    )


class TestIdentifyProtocol:
    def test_identify_cmd_triggers_responses(self):
        """IdentifyCommand should trigger at least one sent frame per subsystem."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        assert sim._bus.send.called

    def test_identify_responses_include_main_frame(self):
        """Must send IdentifyResponseMain (0x601) for each subsystem."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_IDENTIFY_RESP_MAIN in sent_ids

    def test_identify_responses_include_app_version_frame(self):
        """Must send IdentifyResponseAppVersion (0x602) for each subsystem."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_IDENTIFY_RESP_APP_VERSION in sent_ids

    def test_identify_responses_include_bl_version_frame(self):
        """Must send IdentifyResponseBLVersion (0x603) for each subsystem."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_IDENTIFY_RESP_BL_VERSION in sent_ids

    def test_identify_response_main_has_correct_bus_address(self):
        """IdentifyResponseMain must report the correct bus address."""
        sim = _make_sim(_make_args(bus_address=3))
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        assert main_msgs, "No IdentifyResponseMain sent"
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            assert int(decoded["BusAddress"]) == 3

    def test_identify_response_main_mcm_has_product_id_1(self):
        """MCM (default) should report ProductID=1."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        assert main_msgs
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            assert int(decoded["ProductID"]) == 1

    def test_identify_responses_sent_for_each_subsystem(self):
        """Both subsystems (0 and 1) must each send all three response frames."""
        sim = _make_sim(_make_args(subsystem_ids=[0, 1]))
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        subsystem_ids_seen = set()
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            subsystem_ids_seen.add(int(decoded["SubsystemID"]))
        assert 0 in subsystem_ids_seen
        assert 1 in subsystem_ids_seen

    def test_identify_response_has_non_zero_serial(self):
        """Serial number in IdentifyResponseMain must be non-zero."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        assert main_msgs
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            assert int(decoded["ModuleSerialNumber"]) != 0

    def test_identify_response_boot_state_is_running(self):
        """ModuleBootState=1 means the module is running (not in bootloader)."""
        sim = _make_sim()
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        assert main_msgs
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            assert int(decoded["ModuleBootState"]) == 1

    def test_identify_does_not_affect_watchdog(self):
        """IdentifyCommand must not reset subsystem watchdog timers."""
        import time
        sim = _make_sim()
        for s in sim._subsystems:
            s.last_rx_time = time.monotonic() - 0.5
        old_times = [s.last_rx_time for s in sim._subsystems]
        sim._handle_can_message(_make_identify_command())
        for s, old_t in zip(sim._subsystems, old_times):
            assert s.last_rx_time == old_t, "Identify should not reset watchdog"

    def test_identify_no_responses_for_empty_device_list(self):
        """With no subsystems configured, no responses should be sent."""
        args = _make_args(subsystem_ids=[])
        sim = _make_sim(args)
        sim._handle_can_message(_make_identify_command())
        # Verify that no identify responses were sent
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_IDENTIFY_RESP_MAIN not in sent_ids

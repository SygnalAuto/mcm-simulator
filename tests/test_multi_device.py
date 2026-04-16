"""Tests for multi-device support: --devices flag, DeviceSpec parsing,
and correct routing of CAN frames to the right device by bus address.
"""

from __future__ import annotations

import argparse
from unittest.mock import MagicMock

import can
import cantools
from pathlib import Path
import pytest

from mcm_simulator.device_config import (
    DeviceSpec,
    parse_devices_spec,
    PRODUCT_IDS,
)
from mcm_simulator.simulator import (
    McmSimulator,
    _ID_IDENTIFY_CMD,
    _ID_IDENTIFY_RESP_MAIN,
    _parse_args,
)


_DBC_DIR = Path(__file__).parent.parent / "mcm_simulator" / "dbc"


def _load_identify_db() -> cantools.database.Database:
    db = cantools.database.Database()
    db.add_dbc_file(str(_DBC_DIR / "Identify.dbc"))
    return db


def _make_identify_command() -> can.Message:
    return can.Message(
        arbitration_id=_ID_IDENTIFY_CMD,
        data=b"",
        is_extended_id=False,
    )


def _make_sim_with_devices(devices_spec: str) -> McmSimulator:
    """Create a McmSimulator with --devices spec string."""
    devices = parse_devices_spec(devices_spec)
    args = argparse.Namespace(
        can_interface="vcan0",
        watchdog_timeout_ms=200,
        heartbeat_rate_hz=15.0,
        socket_path="/tmp/test-multi.sock",
        devices=devices,
        bus_address=None,
        subsystem_ids=None,
    )
    sim = McmSimulator(args)
    sim._bus = MagicMock(spec=can.BusABC)
    return sim


# ── DeviceSpec & parse_devices_spec ──────────────────────────────────────────


class TestParseDevicesSpec:
    def test_single_mcm(self):
        devices = parse_devices_spec("1:mcm")
        assert len(devices) == 1
        assert devices[0].bus_address == 1
        assert devices[0].product_type == "mcm"
        assert devices[0].product_id == PRODUCT_IDS["mcm"]

    def test_multiple_devices(self):
        devices = parse_devices_spec("1:mcm,2:cb,3:io")
        assert len(devices) == 3
        assert devices[0].bus_address == 1
        assert devices[1].bus_address == 2
        assert devices[2].bus_address == 3

    def test_cb_product_id(self):
        devices = parse_devices_spec("5:cb")
        assert devices[0].product_id == PRODUCT_IDS["cb"]

    def test_io_product_id(self):
        devices = parse_devices_spec("7:io")
        assert devices[0].product_id == PRODUCT_IDS["io"]

    def test_uppercase_type_is_accepted(self):
        devices = parse_devices_spec("1:MCM")
        assert devices[0].product_type == "mcm"

    def test_serial_number_auto_generated(self):
        devices = parse_devices_spec("4:mcm")
        assert devices[0].serial_number != 0

    def test_duplicate_address_raises(self):
        with pytest.raises(ValueError, match="[Dd]uplicate"):
            parse_devices_spec("1:mcm,1:cb")

    def test_unknown_type_raises(self):
        with pytest.raises(ValueError, match="[Uu]nknown"):
            parse_devices_spec("1:unknown")

    def test_missing_colon_raises(self):
        with pytest.raises(ValueError):
            parse_devices_spec("1mcm")

    def test_non_integer_address_raises(self):
        with pytest.raises(ValueError):
            parse_devices_spec("abc:mcm")

    def test_empty_string_raises(self):
        with pytest.raises(ValueError):
            parse_devices_spec("")

    def test_whitespace_around_spec_is_handled(self):
        devices = parse_devices_spec(" 1:mcm , 2:cb ")
        assert len(devices) == 2


class TestDeviceSpec:
    def test_default_subsystem_ids(self):
        spec = DeviceSpec(bus_address=1, product_type="mcm", product_id=1)
        assert spec.subsystem_ids == [0, 1]

    def test_serial_auto_set_if_zero(self):
        spec = DeviceSpec(bus_address=5, product_type="mcm", product_id=1)
        assert spec.serial_number != 0


# ── Multi-device simulator ────────────────────────────────────────────────────


class TestMultiDeviceSimulator:
    def test_two_devices_both_respond_to_identify(self):
        """Both MCM@1 and CB@2 must respond to IdentifyCommand."""
        sim = _make_sim_with_devices("1:mcm,2:cb")
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        bus_addresses_seen = {
            int(db.decode_message(m.arbitration_id, m.data)["BusAddress"])
            for m in main_msgs
        }
        assert 1 in bus_addresses_seen, "MCM@1 did not respond"
        assert 2 in bus_addresses_seen, "CB@2 did not respond"

    def test_three_devices_all_respond_to_identify(self):
        sim = _make_sim_with_devices("1:mcm,2:cb,3:io")
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        bus_addresses_seen = {
            int(db.decode_message(m.arbitration_id, m.data)["BusAddress"])
            for m in main_msgs
        }
        assert {1, 2, 3} == bus_addresses_seen

    def test_device_product_ids_correct(self):
        """Each device type should report its correct ProductID."""
        sim = _make_sim_with_devices("1:mcm,2:cb,3:io")
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        product_by_addr: dict[int, set[int]] = {}
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            addr = int(decoded["BusAddress"])
            pid = int(decoded["ProductID"])
            product_by_addr.setdefault(addr, set()).add(pid)
        assert product_by_addr[1] == {1}  # MCM
        assert product_by_addr[2] == {5}  # CB
        assert product_by_addr[3] == {6}  # IO

    def test_heartbeat_sent_for_all_devices(self):
        """All devices should publish heartbeats."""
        sim = _make_sim_with_devices("1:mcm,2:cb")
        # Run heartbeat for all devices manually
        from mcm_simulator.simulator import _ID_HEARTBEAT
        sim._publish_all_heartbeats()
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert sent_ids.count(_ID_HEARTBEAT) >= 4  # 2 devices × 2 subsystems

    def test_multiple_subsystems_per_device(self):
        """Each device in multi-device mode gets 2 subsystems (0, 1)."""
        sim = _make_sim_with_devices("1:mcm,2:cb")
        db = _load_identify_db()
        sim._handle_can_message(_make_identify_command())
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        # Each device should respond for subsystem 0 and 1
        subs_by_addr: dict[int, set[int]] = {}
        for msg in main_msgs:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
            addr = int(decoded["BusAddress"])
            sub = int(decoded["SubsystemID"])
            subs_by_addr.setdefault(addr, set()).add(sub)
        for addr, subs in subs_by_addr.items():
            assert {0, 1} == subs, f"Device@{addr} missing subsystem responses"


# ── Backwards Compatibility ───────────────────────────────────────────────────


class TestBackwardsCompatibility:
    def test_default_args_no_devices_flag(self, monkeypatch):
        """Without --devices, legacy --bus-address + --subsystem-ids still work."""
        monkeypatch.setattr("sys.argv", ["sim"])
        args = _parse_args()
        sim = McmSimulator(args)
        sim._bus = MagicMock(spec=can.BusABC)
        # Should have 1 device at address 1 with subsystems [0, 1]
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        assert main_msgs, "Legacy mode: MCM at address 1 should respond"
        bus_addrs = {
            int(db.decode_message(m.arbitration_id, m.data)["BusAddress"])
            for m in main_msgs
        }
        assert bus_addrs == {1}

    def test_legacy_bus_address_flag(self, monkeypatch):
        """--bus-address 3 (legacy) creates a single MCM at address 3."""
        monkeypatch.setattr("sys.argv", ["sim", "--bus-address", "3"])
        args = _parse_args()
        sim = McmSimulator(args)
        sim._bus = MagicMock(spec=can.BusABC)
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        bus_addrs = {
            int(db.decode_message(m.arbitration_id, m.data)["BusAddress"])
            for m in main_msgs
        }
        assert bus_addrs == {3}

    def test_devices_flag_overrides_bus_address(self, monkeypatch):
        """--devices flag takes precedence over legacy --bus-address."""
        monkeypatch.setattr(
            "sys.argv", ["sim", "--devices", "2:mcm,3:cb", "--bus-address", "1"]
        )
        args = _parse_args()
        sim = McmSimulator(args)
        sim._bus = MagicMock(spec=can.BusABC)
        sim._handle_can_message(_make_identify_command())
        db = _load_identify_db()
        main_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_IDENTIFY_RESP_MAIN
        ]
        bus_addrs = {
            int(db.decode_message(m.arbitration_id, m.data)["BusAddress"])
            for m in main_msgs
        }
        assert 1 not in bus_addrs, "--bus-address should be ignored when --devices is set"
        assert 2 in bus_addrs
        assert 3 in bus_addrs

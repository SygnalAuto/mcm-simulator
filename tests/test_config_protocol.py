"""Tests for the ConfigGetSet CAN protocol.

ConfigGetSetCommand (0x400) → simulator stores/returns values → ConfigGetSetResponse (0x401)
Uses cantools to encode/decode frames. No real vcan0 needed — Bus is mocked.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from unittest.mock import MagicMock

import can
import cantools

from mcm_simulator.crc8 import apply_crc8, generate_crc8
from mcm_simulator.simulator import (
    McmSimulator,
    _ID_CONFIG_GETSET_CMD,
    _ID_CONFIG_GETSET_RESP,
)

_DBC_DIR = Path(__file__).parent.parent / "mcm_simulator" / "dbc"


def _load_mcm_config_db() -> cantools.database.Database:
    db = cantools.database.Database()
    db.add_dbc_file(str(_DBC_DIR / "mcm_Configuration.dbc"))
    return db


def _make_args(**kwargs) -> argparse.Namespace:
    defaults = dict(
        can_interface="vcan0",
        watchdog_timeout_ms=200,
        heartbeat_rate_hz=15.0,
        bus_address=1,
        subsystem_ids=[0, 1],
        socket_path="/tmp/test-config.sock",
        devices=None,
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


def _make_sim(args: argparse.Namespace | None = None) -> McmSimulator:
    sim = McmSimulator(args or _make_args())
    sim._bus = MagicMock(spec=can.BusABC)
    return sim


def _encode_config_cmd(
    db: cantools.database.Database,
    bus_address: int,
    subsystem_id: int,
    interface_id: int,
    section_signals: dict,
    is_set: bool = False,
) -> can.Message:
    """Encode a ConfigGetSetCommand frame with CRC applied."""
    cmd_msg = db.get_message_by_name("ConfigGetSetCommand")
    signals = {
        "BusAddress": bus_address,
        "SubsystemID": subsystem_id,
        "Config_InterfaceID": interface_id,
        "Config_GetSet": 1 if is_set else 0,
        "CRC": 0,
    }
    signals.update(section_signals)
    raw = bytearray(cmd_msg.encode(signals))
    apply_crc8(raw)
    return can.Message(
        arbitration_id=_ID_CONFIG_GETSET_CMD,
        data=bytes(raw),
        is_extended_id=False,
    )


class TestConfigGetSetBasic:
    def test_get_command_triggers_response(self):
        """ConfigGetSetCommand Get should trigger at least one ConfigGetSetResponse."""
        sim = _make_sim()
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,  # SygnalGlobal
                "Config_ParameterID_SygnalGlobal": 0,  # Bus Address
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
            is_set=False,
        )
        sim._handle_can_message(cmd)
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_CONFIG_GETSET_RESP in sent_ids

    def test_set_command_triggers_response(self):
        """ConfigGetSetCommand Set should trigger ConfigGetSetResponse."""
        sim = _make_sim()
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 42,
            },
            is_set=True,
        )
        sim._handle_can_message(cmd)
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_CONFIG_GETSET_RESP in sent_ids

    def test_response_has_correct_crc(self):
        """Every ConfigGetSetResponse must have correct CRC8 in byte 7."""
        sim = _make_sim()
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
        )
        sim._handle_can_message(cmd)
        resp_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
        ]
        assert resp_msgs
        for msg in resp_msgs:
            raw = bytearray(msg.data)
            assert generate_crc8(raw) == raw[7], "CRC mismatch in ConfigGetSetResponse"

    def test_both_subsystems_respond(self):
        """Both subsystems (0 and 1) must each send a ConfigGetSetResponse."""
        sim = _make_sim(_make_args(subsystem_ids=[0, 1]))
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
        )
        sim._handle_can_message(cmd)
        resp_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
        ]
        assert len(resp_msgs) == 2, f"Expected 2 responses (one per subsystem), got {len(resp_msgs)}"

    def test_get_returns_default_zero_when_unset(self):
        """Get for an unset parameter should return 0."""
        sim = _make_sim()
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
        )
        sim._handle_can_message(cmd)
        resp_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
        ]
        assert resp_msgs
        resp_db = _load_mcm_config_db()
        decoded = resp_db.decode_message(resp_msgs[0].arbitration_id, resp_msgs[0].data)
        value = decoded.get("Config_Value_SygnalGlobal_UINT7", None)
        assert value is not None, "Config_Value_SygnalGlobal_UINT7 not in response"
        assert int(value) == 0


class TestConfigSetAndGet:
    def test_set_stores_value_returned_on_get(self):
        """Set a value, then Get it — must return the set value."""
        sim = _make_sim()
        db = _load_mcm_config_db()

        # SET Bus Address param to 7
        set_cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 7,
            },
            is_set=True,
        )
        sim._handle_can_message(set_cmd)
        sim._bus.reset_mock()

        # GET Bus Address param
        get_cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,  # value ignored for Get
            },
            is_set=False,
        )
        sim._handle_can_message(get_cmd)
        resp_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
        ]
        assert resp_msgs
        decoded = db.decode_message(resp_msgs[0].arbitration_id, resp_msgs[0].data)
        value = decoded.get("Config_Value_SygnalGlobal_UINT7", None)
        assert int(value) == 7, f"Expected 7, got {value}"

    def test_set_response_echoes_set_value(self):
        """Set response must echo back the value that was set."""
        sim = _make_sim()
        db = _load_mcm_config_db()

        set_cmd = _encode_config_cmd(
            db,
            bus_address=1,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 99,
            },
            is_set=True,
        )
        sim._handle_can_message(set_cmd)
        resp_msgs = [
            c[0][0]
            for c in sim._bus.send.call_args_list
            if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
        ]
        assert resp_msgs
        decoded = db.decode_message(resp_msgs[0].arbitration_id, resp_msgs[0].data)
        value = decoded.get("Config_Value_SygnalGlobal_UINT7", None)
        assert int(value) == 99

    def test_values_independent_per_interface(self):
        """Values set on interface 0 and interface 1 are stored independently."""
        sim = _make_sim()
        db = _load_mcm_config_db()

        # Assume Control section (6), Control Type param (0), interface 0 and 1
        # Use SygnalGlobal which has interface_id=0 conceptually; test with explicit iface
        for iface_id, expected_val in [(0, 10), (1, 20)]:
            set_cmd = _encode_config_cmd(
                db,
                bus_address=1,
                subsystem_id=0,
                interface_id=iface_id,
                section_signals={
                    "Config_SectionID": 0,
                    "Config_ParameterID_SygnalGlobal": 0,
                    "Config_Value_SygnalGlobal_UINT7": expected_val,
                },
                is_set=True,
            )
            sim._handle_can_message(set_cmd)
        sim._bus.reset_mock()

        for iface_id, expected_val in [(0, 10), (1, 20)]:
            get_cmd = _encode_config_cmd(
                db,
                bus_address=1,
                subsystem_id=0,
                interface_id=iface_id,
                section_signals={
                    "Config_SectionID": 0,
                    "Config_ParameterID_SygnalGlobal": 0,
                    "Config_Value_SygnalGlobal_UINT7": 0,
                },
                is_set=False,
            )
            sim._handle_can_message(get_cmd)
            resp_msgs = [
                c[0][0]
                for c in sim._bus.send.call_args_list
                if c[0][0].arbitration_id == _ID_CONFIG_GETSET_RESP
            ]
            decoded = db.decode_message(resp_msgs[-2].arbitration_id, resp_msgs[-2].data)
            value = decoded.get("Config_Value_SygnalGlobal_UINT7")
            assert int(value) == expected_val, f"Interface {iface_id}: expected {expected_val}, got {value}"


class TestConfigBusAddressFiltering:
    def test_command_for_wrong_address_ignored(self):
        """Config command addressed to a different device must be ignored."""
        sim = _make_sim(_make_args(bus_address=1))
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=2,  # Wrong address
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
        )
        sim._handle_can_message(cmd)
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_CONFIG_GETSET_RESP not in sent_ids

    def test_command_for_correct_address_processed(self):
        """Config command addressed to this device's bus address must be processed."""
        sim = _make_sim(_make_args(bus_address=3))
        db = _load_mcm_config_db()
        cmd = _encode_config_cmd(
            db,
            bus_address=3,
            subsystem_id=0,
            interface_id=0,
            section_signals={
                "Config_SectionID": 0,
                "Config_ParameterID_SygnalGlobal": 0,
                "Config_Value_SygnalGlobal_UINT7": 0,
            },
        )
        sim._handle_can_message(cmd)
        sent_ids = [c[0][0].arbitration_id for c in sim._bus.send.call_args_list]
        assert _ID_CONFIG_GETSET_RESP in sent_ids


class TestConfigStore:
    def test_config_store_get_returns_default(self):
        from mcm_simulator.config_store import ConfigStore
        store = ConfigStore()
        value = store.get(1, 0, 0, 0, 0)
        assert value == 0.0

    def test_config_store_set_then_get(self):
        from mcm_simulator.config_store import ConfigStore
        store = ConfigStore()
        store.set(1, 0, 0, 9, 3, 1.5)
        assert store.get(1, 0, 0, 9, 3) == 1.5

    def test_config_store_keys_are_independent(self):
        from mcm_simulator.config_store import ConfigStore
        store = ConfigStore()
        store.set(1, 0, 0, 0, 0, 10.0)
        store.set(1, 1, 0, 0, 0, 20.0)  # different subsystem
        store.set(2, 0, 0, 0, 0, 30.0)  # different bus address
        assert store.get(1, 0, 0, 0, 0) == 10.0
        assert store.get(1, 1, 0, 0, 0) == 20.0
        assert store.get(2, 0, 0, 0, 0) == 30.0

"""Tests for CI-facing Unix socket injection hooks.

Exercises set-state, inject-fault, emit-error, and query-detailed commands
added for deterministic CI testing of interfacing applications.
"""

from __future__ import annotations

import asyncio
import json
import os
from unittest.mock import MagicMock

import pytest

from mcm_simulator.simulator import (
    McmSimulator,
    SubsystemState,
    SystemState,
    _ID_ERROR_STATUS,
    _ID_FAULT_INCREMENT,
    _load_dbc,
)
from mcm_simulator.device_config import DeviceSpec
from mcm_simulator.simulator import _DeviceNode


def _make_sim(socket_path: str) -> McmSimulator:
    db = _load_dbc()
    args = MagicMock()
    args.bus_address = 1
    args.watchdog_timeout_ms = 200
    args.heartbeat_rate_hz = 15.0
    args.report_rate_hz = 10.0
    args.subsystem_ids = [0, 1]
    args.devices = None
    args.socket_path = socket_path

    sim = McmSimulator.__new__(McmSimulator)
    sim._db = db
    sim._bus = MagicMock()
    sim._args = args

    subs = [
        SubsystemState(subsystem_id=0, bus_address=1, watchdog_timeout_s=0.2),
        SubsystemState(subsystem_id=1, bus_address=1, watchdog_timeout_s=0.2),
    ]
    for sub in subs:
        sub.state = SystemState.HUMAN_CONTROL
        sub.on_fault_increment = sim._publish_fault_increment

    spec = DeviceSpec(bus_address=1, product_type="mcm", product_id=1)
    node = _DeviceNode(spec=spec, subsystems=subs)
    sim._device_nodes = [node]
    sim._subsystems = subs

    return sim


async def _send_cmd(socket_path: str, cmd: str) -> str:
    reader, writer = await asyncio.open_unix_connection(socket_path)
    writer.write((cmd + "\n").encode())
    await writer.drain()
    response = await asyncio.wait_for(reader.readline(), timeout=2.0)
    writer.close()
    await writer.wait_closed()
    return response.decode().strip()


@pytest.fixture
def tmp_socket(tmp_path):
    return str(tmp_path / "test-ci.sock")


@pytest.fixture
def running_sim(tmp_socket):
    """Start a sim with its socket server running."""
    sim = _make_sim(tmp_socket)

    async def _serve():
        if os.path.exists(tmp_socket):
            os.unlink(tmp_socket)
        server = await asyncio.start_unix_server(
            sim._handle_socket_client, path=tmp_socket
        )
        async with server:
            await server.serve_forever()

    loop = asyncio.new_event_loop()

    import threading
    import time

    def run_server():
        asyncio.set_event_loop(loop)
        loop.run_forever()

    t = threading.Thread(target=run_server, daemon=True)
    t.start()

    asyncio.run_coroutine_threadsafe(_serve(), loop)
    time.sleep(0.05)  # let server bind

    yield sim, tmp_socket

    loop.call_soon_threadsafe(loop.stop)


def _send_sync(socket_path: str, cmd: str) -> str:
    return asyncio.run(_send_cmd(socket_path, cmd))


class TestSetState:
    def test_set_state_fail_hard(self, running_sim) -> None:
        sim, sock = running_sim
        resp = _send_sync(sock, "set-state 0 FAIL_HARD")
        assert resp == "ok"
        assert sim._subsystems[0].state == SystemState.FAIL_HARD

    def test_set_state_human_control(self, running_sim) -> None:
        sim, sock = running_sim
        sim._subsystems[0].state = SystemState.FAIL_HARD
        resp = _send_sync(sock, "set-state 0 HUMAN_CONTROL")
        assert resp == "ok"
        assert sim._subsystems[0].state == SystemState.HUMAN_CONTROL

    def test_set_state_all_six_values(self, running_sim) -> None:
        sim, sock = running_sim
        for name in ("HUMAN_CONTROL", "MCM_CONTROL", "FAIL_HARD",
                     "FAIL_OP_1", "FAIL_OP_2", "HUMAN_OVERRIDE"):
            resp = _send_sync(sock, f"set-state 0 {name}")
            assert resp == "ok", f"set-state 0 {name} failed: {resp}"

    def test_set_state_unknown_name_returns_error(self, running_sim) -> None:
        _, sock = running_sim
        resp = _send_sync(sock, "set-state 0 BANANA")
        assert resp.startswith("error:")

    def test_set_state_bad_subsystem_returns_error(self, running_sim) -> None:
        _, sock = running_sim
        resp = _send_sync(sock, "set-state 99 FAIL_HARD")
        assert resp.startswith("error:")


class TestInjectFault:
    def test_inject_fault_increments_counter(self, running_sim) -> None:
        sim, sock = running_sim
        before = sim._subsystems[0].fault_count
        resp = _send_sync(sock, "inject-fault 0 0x30")
        assert resp == "ok"
        assert sim._subsystems[0].fault_count == before + 1
        assert sim._subsystems[0].fault_cause == 0x30

    def test_inject_fault_emits_fault_increment_frame(self, running_sim) -> None:
        sim, sock = running_sim
        sim._bus.send.reset_mock()
        _send_sync(sock, "inject-fault 0 0x01")
        sent_ids = [call.args[0].arbitration_id for call in sim._bus.send.call_args_list]
        assert _ID_FAULT_INCREMENT in sent_ids

    def test_inject_fault_bad_subsystem_returns_error(self, running_sim) -> None:
        _, sock = running_sim
        resp = _send_sync(sock, "inject-fault 99 0x01")
        assert resp.startswith("error:")


class TestEmitError:
    def test_emit_error_puts_frame_on_bus(self, running_sim) -> None:
        sim, sock = running_sim
        sim._bus.send.reset_mock()
        resp = _send_sync(sock, "emit-error 0 1 0x60")
        assert resp == "ok"
        sent_ids = [call.args[0].arbitration_id for call in sim._bus.send.call_args_list]
        assert _ID_ERROR_STATUS in sent_ids

    def test_emit_error_correct_type_and_canid(self, running_sim) -> None:
        sim, sock = running_sim
        db = sim._db
        sim._bus.send.reset_mock()
        _send_sync(sock, "emit-error 0 6 0x160")
        error_frames = [
            call.args[0] for call in sim._bus.send.call_args_list
            if call.args[0].arbitration_id == _ID_ERROR_STATUS
        ]
        assert len(error_frames) == 1
        decoded = db.decode_message(_ID_ERROR_STATUS, error_frames[0].data, decode_choices=False)
        assert decoded["Error_Type"] == 6
        assert decoded["Error_CANID"] == 0x160

    def test_emit_error_bad_subsystem_returns_error(self, running_sim) -> None:
        _, sock = running_sim
        resp = _send_sync(sock, "emit-error 99 1 0x60")
        assert resp.startswith("error:")


class TestQueryDetailed:
    def test_query_detailed_returns_valid_json(self, running_sim) -> None:
        sim, sock = running_sim
        resp = _send_sync(sock, "query-detailed")
        data = json.loads(resp)
        assert isinstance(data, dict)

    def test_query_detailed_has_required_fields(self, running_sim) -> None:
        sim, sock = running_sim
        resp = _send_sync(sock, "query-detailed")
        data = json.loads(resp)
        for sub_id_str, sub_data in data.items():
            for field in ("state", "state_int", "fault_count",
                          "fault_cause", "last_rx_age_ms", "counter", "estop_latched"):
                assert field in sub_data, f"Missing field {field!r} for sub {sub_id_str}"

    def test_query_detailed_reflects_state_change(self, running_sim) -> None:
        sim, sock = running_sim
        _send_sync(sock, "set-state 0 FAIL_HARD")
        resp = _send_sync(sock, "query-detailed")
        data = json.loads(resp)
        assert data["0"]["state"] == "FAIL_HARD"
        assert data["0"]["state_int"] == 254

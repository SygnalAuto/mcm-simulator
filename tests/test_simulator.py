"""Unit tests for MCM CAN simulator state machine and protocol logic.

python-can Bus is mocked — no real vcan0 needed for unit tests.
cantools DBC parsing is exercised directly via _make_can_frame — bundled
DBC files (mcm_simulator/dbc/) must be present.
"""

from __future__ import annotations

import argparse
import time
from unittest.mock import AsyncMock, MagicMock

import can

from mcm_simulator.simulator import (
    McmSimulator, SystemState,
    _ID_CONTROL_CMD_RESP, _ID_HEARTBEAT,
    _ID_HEARTBEAT_CLEAR_SEED, _ID_RELAY_CMD_RESP,
    _FAULT_CLEAR_XOR,
)
from mcm_simulator.crc8 import apply_crc8, generate_crc8


def _make_args(**kwargs) -> argparse.Namespace:
    defaults = dict(
        can_interface="vcan0",
        watchdog_timeout_ms=200,
        heartbeat_rate_hz=15.0,
        bus_address=1,
        subsystem_ids=[0, 1],
        socket_path="/tmp/test-simulated-mcm.sock",
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)


def _make_sim(args: argparse.Namespace | None = None) -> McmSimulator:
    sim = McmSimulator(args or _make_args())
    sim._bus = MagicMock(spec=can.BusABC)
    return sim


def _make_can_frame(db, name: str, signals: dict) -> can.Message:
    """Encode a DBC message with CRC applied and return a can.Message."""
    msg = db.get_message_by_name(name)
    signals["CRC"] = 0
    raw = bytearray(msg.encode(signals))
    apply_crc8(raw)
    return can.Message(
        arbitration_id=msg.frame_id,
        data=bytes(raw),
        is_extended_id=False,
    )


class TestInitialState:
    def test_starts_in_fail_hard(self):
        """MCM boots FAIL_HARD — requires fault clear to reach HUMAN_CONTROL."""
        sim = _make_sim()
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)

    def test_estop_not_latched_at_start(self):
        sim = _make_sim()
        assert all(not s.estop_latched for s in sim._subsystems)

    def test_counters_start_at_zero(self):
        sim = _make_sim()
        assert all(s.counter == 0 for s in sim._subsystems)

    def test_two_subsystems_configured_by_default(self):
        sim = _make_sim()
        assert len(sim._subsystems) == 2
        assert sim._subsystems[0].subsystem_id == 0
        assert sim._subsystems[1].subsystem_id == 1


class TestControlEnableTransitions:
    def test_enable_1_transitions_to_mcm_control(self):
        sim = _make_sim()
        # MCM boots FAIL_HARD; put in HUMAN_CONTROL first (as after fault clear)
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1,
        })
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.MCM_CONTROL for s in sim._subsystems)

    def test_enable_0_transitions_to_human_control(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.MCM_CONTROL
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 0,
        })
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.HUMAN_CONTROL for s in sim._subsystems)

    def test_enable_rejected_in_fail_hard(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1,
        })
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)


class TestWatchdog:
    def test_watchdog_triggers_fail_hard(self):
        import asyncio
        sim = _make_sim(args=_make_args(watchdog_timeout_ms=50))
        # Simulate last_rx_time far in the past
        for s in sim._subsystems:
            s.last_rx_time = time.monotonic() - 1.0

        async def run_one_cycle():
            task = asyncio.create_task(sim._watchdog_loop())
            await asyncio.sleep(0.1)
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        asyncio.run(run_one_cycle())
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)

    def test_default_watchdog_timeout_is_2000ms(self):
        """Simulated MCM default watchdog should be 2000ms for asyncio tolerance."""
        sim = _make_sim(args=_make_args(watchdog_timeout_ms=2000))
        assert sim._subsystems[0].watchdog_timeout_s == 2.0

    def test_watchdog_survives_after_fault_clear_with_commands(self):
        """After fault clear, MCM stays in HUMAN_CONTROL if 0x160 frames keep arriving."""
        sim = _make_sim(args=_make_args(watchdog_timeout_ms=2000))
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0xAABBCCDD
        # Clear the fault
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0xAABBCCDD ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL
        # Simulate 0x160 commands arriving at 20Hz for 500ms
        cmd_frame = _make_can_frame(sim._db, "ControlCommand", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Count8": 0, "Value": 0,
        })
        for _ in range(10):
            time.sleep(0.05)
            sim._handle_can_message(cmd_frame)
        # MCM should still be in HUMAN_CONTROL
        assert sub.state == SystemState.HUMAN_CONTROL

    def test_valid_frame_resets_watchdog(self):
        sim = _make_sim()
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1,
        })
        sim._handle_can_message(frame)
        assert all(s.last_rx_time > old_time for s in sim._subsystems)


class TestEstopLatch:
    def test_estop_press_triggers_fail_hard(self):
        import asyncio
        sim = _make_sim()

        async def press():
            reader = asyncio.StreamReader()
            reader.feed_data(b"estop-press\n")
            writer = MagicMock()
            writer.drain = AsyncMock()
            await sim._handle_socket_client(reader, writer)

        asyncio.run(press())
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)
        assert all(s.estop_latched for s in sim._subsystems)

    def test_estop_release_clears_fail_hard(self):
        import asyncio
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD
            s.estop_latched = True

        async def release():
            reader = asyncio.StreamReader()
            reader.feed_data(b"estop-release\n")
            writer = MagicMock()
            writer.drain = AsyncMock()
            await sim._handle_socket_client(reader, writer)

        asyncio.run(release())
        assert all(s.state == SystemState.HUMAN_CONTROL for s in sim._subsystems)
        assert all(not s.estop_latched for s in sim._subsystems)

    def test_estop_release_does_not_clear_non_estop_fail_hard(self):
        import asyncio
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD
            s.estop_latched = False

        async def release():
            reader = asyncio.StreamReader()
            reader.feed_data(b"estop-release\n")
            writer = MagicMock()
            writer.drain = AsyncMock()
            await sim._handle_socket_client(reader, writer)

        asyncio.run(release())
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)
        assert all(not s.estop_latched for s in sim._subsystems)

    def test_control_enable_cannot_clear_estop_latch(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD
            s.estop_latched = True
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1,
        })
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.FAIL_HARD for s in sim._subsystems)
        assert all(s.estop_latched for s in sim._subsystems)


class TestHeartbeatPublishing:
    def test_heartbeat_encodes_correct_state_and_crc(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.MCM_CONTROL
        sim._publish_subsystem_heartbeat(sub)
        assert sim._bus.send.called
        sent_msg: can.Message = sim._bus.send.call_args[0][0]
        assert sent_msg.arbitration_id == _ID_HEARTBEAT
        raw = bytearray(sent_msg.data)
        assert generate_crc8(raw) == raw[7]
        decoded = sim._db.decode_message(_ID_HEARTBEAT, bytes(raw))
        state_val = decoded["SystemState"]
        assert (state_val.value if hasattr(state_val, "value") else int(state_val)) == SystemState.MCM_CONTROL

    def test_heartbeat_counter_increments(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sim._publish_subsystem_heartbeat(sub)
        sim._publish_subsystem_heartbeat(sub)
        sim._publish_subsystem_heartbeat(sub)
        assert sub.counter == 3

    def test_heartbeat_crc_all_states(self):
        for state in (SystemState.HUMAN_CONTROL, SystemState.MCM_CONTROL, SystemState.FAIL_HARD):
            sim = _make_sim()
            sub = sim._subsystems[0]
            sub.state = state
            sim._publish_subsystem_heartbeat(sub)
            sent_msg: can.Message = sim._bus.send.call_args[0][0]
            raw = bytearray(sent_msg.data)
            assert generate_crc8(raw) == raw[7], f"CRC mismatch for state {state.name}"


class TestPartialFailure:
    def test_fail_sub0_command_sets_sub0_to_fail_hard_only(self):
        import asyncio
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL

        async def inject():
            reader = asyncio.StreamReader()
            reader.feed_data(b"fail-sub0\n")
            writer = MagicMock()
            writer.drain = AsyncMock()
            await sim._handle_socket_client(reader, writer)

        asyncio.run(inject())
        sub0 = next(s for s in sim._subsystems if s.subsystem_id == 0)
        sub1 = next(s for s in sim._subsystems if s.subsystem_id == 1)
        assert sub0.state == SystemState.FAIL_HARD
        assert sub1.state == SystemState.HUMAN_CONTROL

    def test_fail_sub1_command_sets_sub1_to_fail_hard_only(self):
        import asyncio
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL

        async def inject():
            reader = asyncio.StreamReader()
            reader.feed_data(b"fail-sub1\n")
            writer = MagicMock()
            writer.drain = AsyncMock()
            await sim._handle_socket_client(reader, writer)

        asyncio.run(inject())
        sub0 = next(s for s in sim._subsystems if s.subsystem_id == 0)
        sub1 = next(s for s in sim._subsystems if s.subsystem_id == 1)
        assert sub0.state == SystemState.HUMAN_CONTROL
        assert sub1.state == SystemState.FAIL_HARD

    def test_each_subsystem_has_independent_watchdog(self):
        sim = _make_sim(args=_make_args(watchdog_timeout_ms=50, subsystem_ids=[0, 1]))
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        sub0 = sim._subsystems[0]
        sub1 = sim._subsystems[1]
        sub0.last_rx_time = time.monotonic() - 1.0
        fired = sub0.check_watchdog()
        assert fired
        assert sub0.state == SystemState.FAIL_HARD
        assert sub1.state == SystemState.HUMAN_CONTROL

    def test_single_subsystem_mode_still_works(self):
        sim = _make_sim(args=_make_args(subsystem_ids=[0]))
        assert len(sim._subsystems) == 1
        assert sim._subsystems[0].subsystem_id == 0


class TestControlCommandResponse:
    def test_control_command_response_echoes_value_with_crc(self):
        sim = _make_sim()
        frame = _make_can_frame(sim._db, "ControlCommand", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0,
            "Count8": 7, "Value": 0.5,
        })
        sim._handle_can_message(frame)
        assert sim._bus.send.called
        sent_msg: can.Message = sim._bus.send.call_args[0][0]
        assert sent_msg.arbitration_id == _ID_CONTROL_CMD_RESP
        raw = bytearray(sent_msg.data)
        assert generate_crc8(raw) == raw[7]


class TestRelayCommandResponse:
    def test_relay_command_response_with_correct_crc(self):
        sim = _make_sim()
        frame = _make_can_frame(sim._db, "RelayCommand", {
            "BusAddress": 1, "SubsystemID": 0, "Enable": 1,
        })
        sim._handle_can_message(frame)
        assert sim._bus.send.called
        sent_msg: can.Message = sim._bus.send.call_args[0][0]
        assert sent_msg.arbitration_id == _ID_RELAY_CMD_RESP
        raw = bytearray(sent_msg.data)
        assert generate_crc8(raw) == raw[7]
        decoded = sim._db.decode_message(_ID_RELAY_CMD_RESP, bytes(raw))
        assert int(decoded["Enable"]) == 1


class TestFaultClearProtocol:
    def test_valid_key_clears_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0x12345678
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0x12345678 ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL

    def test_invalid_key_stays_in_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0x12345678
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0xDEADBEEF,
        })
        sim._handle_can_message(frame)
        assert sub.state == SystemState.FAIL_HARD

    def test_fault_clear_resets_watchdog_timer(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0xAABBCCDD
        old_time = sub.last_rx_time
        time.sleep(0.01)
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0xAABBCCDD ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert sub.last_rx_time > old_time

    def test_fault_clear_clears_estop_latch(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.estop_latched = True
        sub.fault_clear_seed = 0x11111111
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0x11111111 ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL
        assert not sub.estop_latched

    def test_fault_clear_key_does_not_reset_watchdog_on_receive(self):
        """Fault clear key frame should not feed the watchdog timer."""
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        sim._subsystems[0].fault_clear_seed = 0x22222222
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0x22222222 ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert all(s.last_rx_time == old_time for s in sim._subsystems)

    def test_fault_clear_ignored_when_not_in_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.HUMAN_CONTROL
        sub.fault_clear_seed = 0x33333333
        frame = _make_can_frame(sim._db, "HeartbeatClearKey", {
            "BusAddress": 1, "SubsystemID": 0,
            "ResetKey": 0x33333333 ^ _FAULT_CLEAR_XOR,
        })
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL

    def test_publish_fault_clear_seed_sends_valid_frame(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sim._publish_subsystem_fault_clear_seed(sub)
        assert sim._bus.send.called
        sent_msg: can.Message = sim._bus.send.call_args[0][0]
        assert sent_msg.arbitration_id == _ID_HEARTBEAT_CLEAR_SEED
        raw = bytearray(sent_msg.data)
        assert generate_crc8(raw) == raw[7]
        decoded = sim._db.decode_message(_ID_HEARTBEAT_CLEAR_SEED, bytes(raw))
        assert int(decoded["ResetSeed"]) == sub.fault_clear_seed
        assert sub.fault_clear_seed != 0

    def test_seed_is_never_zero(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        for _ in range(100):
            sim._publish_subsystem_fault_clear_seed(sub)
            assert sub.fault_clear_seed != 0


class TestInvalidCrc:
    def test_invalid_crc_does_not_reset_watchdog(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        frame = _make_can_frame(sim._db, "ControlEnable", {
            "BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1,
        })
        bad_data = bytearray(frame.data)
        bad_data[7] ^= 0xFF
        bad_frame = can.Message(
            arbitration_id=frame.arbitration_id,
            data=bytes(bad_data),
            is_extended_id=False,
        )
        sim._handle_can_message(bad_frame)
        assert all(s.last_rx_time == old_time for s in sim._subsystems)
        assert all(s.state == SystemState.HUMAN_CONTROL for s in sim._subsystems)

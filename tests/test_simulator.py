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
    McmSimulator,
    SystemState,
    _ID_CONTROL_CMD_RESP,
    _ID_ERROR_STATUS,
    _ID_HEARTBEAT,
    _ID_HEARTBEAT_CLEAR_SEED,
    _ID_RELAY_CMD_RESP,
    _FAULT_CLEAR_XOR,
    _parse_args,
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


class TestSystemStateEnum:
    def test_enum_has_six_members(self) -> None:
        assert len(SystemState) == 6

    def test_enum_values(self) -> None:
        assert SystemState.HUMAN_CONTROL == 0
        assert SystemState.MCM_CONTROL == 1
        assert SystemState.FAIL_OPERATIONAL_1 == 241
        assert SystemState.FAIL_OPERATIONAL_2 == 242
        assert SystemState.HUMAN_OVERRIDE == 253
        assert SystemState.FAIL_HARD == 254

    def test_new_values_in_heartbeat_dbc_range(self) -> None:
        from mcm_simulator.simulator import _load_dbc
        db = _load_dbc()
        hb = db.get_message_by_name("Heartbeat")
        ss_signal = hb.get_signal_by_name("SystemState")
        # All six SystemState values must fit in the SystemState signal's bit-width
        for state in SystemState:
            assert 0 <= int(state) <= (1 << ss_signal.length) - 1, f"{state} out of range"


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
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.MCM_CONTROL for s in sim._subsystems)

    def test_enable_0_transitions_to_human_control(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.MCM_CONTROL
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 0,
            },
        )
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.HUMAN_CONTROL for s in sim._subsystems)

    def test_enable_rejected_in_fail_hard(self):
        sim = _make_sim()
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
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

    def test_default_watchdog_timeout_is_200ms(self):
        """Default watchdog matches real MCM firmware (200ms per sygnal_docs v2.1.1)."""
        import sys
        from unittest.mock import patch
        with patch.object(sys, "argv", ["mcm-simulator"]):
            args = _parse_args()
        assert args.watchdog_timeout_ms == 200

    def test_watchdog_survives_after_fault_clear_with_commands(self):
        """After fault clear, MCM stays in HUMAN_CONTROL if 0x160 frames keep arriving."""
        sim = _make_sim(args=_make_args(watchdog_timeout_ms=2000))
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0xAABBCCDD
        # Clear the fault
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0xAABBCCDD ^ _FAULT_CLEAR_XOR,
            },
        )
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL
        # Simulate 0x160 commands arriving at 20Hz for 500ms
        cmd_frame = _make_can_frame(
            sim._db,
            "ControlCommand",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Count8": 0,
                "Value": 0,
            },
        )
        for _ in range(10):
            time.sleep(0.05)
            sim._handle_can_message(cmd_frame)
        # MCM should still be in HUMAN_CONTROL
        assert sub.state == SystemState.HUMAN_CONTROL

    def test_valid_frame_resets_watchdog(self):
        sim = _make_sim()
        # Must be in HUMAN_CONTROL — in FAIL_HARD control frames are rejected and
        # watchdog is intentionally NOT fed (per Task 5 docs compliance fix).
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
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
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
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
        assert (
            state_val.value if hasattr(state_val, "value") else int(state_val)
        ) == SystemState.MCM_CONTROL

    def test_heartbeat_counter_increments(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sim._publish_subsystem_heartbeat(sub)
        sim._publish_subsystem_heartbeat(sub)
        sim._publish_subsystem_heartbeat(sub)
        assert sub.counter == 3

    def test_heartbeat_crc_all_states(self):
        for state in (
            SystemState.HUMAN_CONTROL,
            SystemState.MCM_CONTROL,
            SystemState.FAIL_HARD,
        ):
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
        # Must be in HUMAN_CONTROL — FAIL_HARD rejects control frames
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        frame = _make_can_frame(
            sim._db,
            "ControlCommand",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Count8": 7,
                "Value": 0.5,
            },
        )
        sim._handle_can_message(frame)
        # Responses are emitted per-subsystem; find first ControlCommandResponse
        resp_msgs = [
            call.args[0] for call in sim._bus.send.call_args_list
            if call.args[0].arbitration_id == _ID_CONTROL_CMD_RESP
        ]
        assert len(resp_msgs) == len(sim._subsystems)
        raw = bytearray(resp_msgs[0].data)
        assert generate_crc8(raw) == raw[7]


class TestRelayCommandResponse:
    def test_relay_command_response_with_correct_crc(self):
        sim = _make_sim()
        # Must be in HUMAN_CONTROL — FAIL_HARD rejects relay frames
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        frame = _make_can_frame(
            sim._db,
            "RelayCommand",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "Enable": 1,
            },
        )
        sim._handle_can_message(frame)
        resp_msgs = [
            call.args[0] for call in sim._bus.send.call_args_list
            if call.args[0].arbitration_id == _ID_RELAY_CMD_RESP
        ]
        assert len(resp_msgs) == len(sim._subsystems)
        raw = bytearray(resp_msgs[0].data)
        assert generate_crc8(raw) == raw[7]
        decoded = sim._db.decode_message(_ID_RELAY_CMD_RESP, bytes(raw))
        assert int(decoded["Enable"]) == 1


class TestFaultClearProtocol:
    def test_valid_key_clears_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0x12345678
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0x12345678 ^ _FAULT_CLEAR_XOR,
            },
        )
        sim._handle_can_message(frame)
        assert sub.state == SystemState.HUMAN_CONTROL

    def test_invalid_key_stays_in_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0x12345678
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0xDEADBEEF,
            },
        )
        sim._handle_can_message(frame)
        assert sub.state == SystemState.FAIL_HARD

    def test_fault_clear_resets_watchdog_timer(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.fault_clear_seed = 0xAABBCCDD
        old_time = sub.last_rx_time
        time.sleep(0.01)
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0xAABBCCDD ^ _FAULT_CLEAR_XOR,
            },
        )
        sim._handle_can_message(frame)
        assert sub.last_rx_time > old_time

    def test_fault_clear_clears_estop_latch(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.FAIL_HARD
        sub.estop_latched = True
        sub.fault_clear_seed = 0x11111111
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0x11111111 ^ _FAULT_CLEAR_XOR,
            },
        )
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
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0x22222222 ^ _FAULT_CLEAR_XOR,
            },
        )
        sim._handle_can_message(frame)
        assert all(s.last_rx_time == old_time for s in sim._subsystems)

    def test_fault_clear_ignored_when_not_in_fail_hard(self):
        sim = _make_sim()
        sub = sim._subsystems[0]
        sub.state = SystemState.HUMAN_CONTROL
        sub.fault_clear_seed = 0x33333333
        frame = _make_can_frame(
            sim._db,
            "HeartbeatClearKey",
            {
                "BusAddress": 1,
                "SubsystemID": 0,
                "ResetKey": 0x33333333 ^ _FAULT_CLEAR_XOR,
            },
        )
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
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 1,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
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


class TestBusAddressFiltering:
    def test_frame_with_wrong_bus_address_is_dropped(self):
        """Frame addressed to a different MCM must not change state."""
        sim = _make_sim(args=_make_args(bus_address=1))
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        # BusAddress=2 — this MCM is address 1, should be ignored
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 2,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
        sim._handle_can_message(frame)
        assert all(s.state == SystemState.HUMAN_CONTROL for s in sim._subsystems)

    def test_foreign_address_frame_does_not_feed_watchdog(self):
        """Frame addressed to a different MCM must not reset this MCM's watchdog."""
        sim = _make_sim(args=_make_args(bus_address=1))
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 2,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
        sim._handle_can_message(frame)
        assert all(s.last_rx_time == old_time for s in sim._subsystems)

    def test_frame_with_correct_bus_address_is_processed(self):
        """Frame addressed to this MCM's bus address is processed normally."""
        sim = _make_sim(args=_make_args(bus_address=3))
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL
        old_time = time.monotonic() - 0.5
        for s in sim._subsystems:
            s.last_rx_time = old_time
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {
                "BusAddress": 3,
                "SubSystemID": 0,
                "InterfaceID": 0,
                "Enable": 1,
            },
        )
        sim._handle_can_message(frame)
        # Watchdog must be fed and state transitions happen
        assert all(s.last_rx_time > old_time for s in sim._subsystems)
        assert all(s.state == SystemState.MCM_CONTROL for s in sim._subsystems)


class TestSocketPathDefault:
    def test_default_socket_path_includes_bus_address(self, monkeypatch):
        """Default socket path must be /tmp/simulated-mcm-{bus_address}.sock."""
        monkeypatch.setattr("sys.argv", ["sim", "--bus-address", "2"])
        args = _parse_args()
        assert args.socket_path == "/tmp/simulated-mcm-2.sock"

    def test_default_socket_path_uses_bus_address_1_when_default(self, monkeypatch):
        """Default bus address 1 produces /tmp/simulated-mcm-1.sock."""
        monkeypatch.setattr("sys.argv", ["sim"])
        args = _parse_args()
        assert args.socket_path == "/tmp/simulated-mcm-1.sock"

    def test_explicit_socket_path_overrides_default(self, monkeypatch):
        """Explicitly passing --socket-path must take precedence over the dynamic default."""
        monkeypatch.setattr(
            "sys.argv",
            ["sim", "--bus-address", "3", "--socket-path", "/run/my-mcm.sock"],
        )
        args = _parse_args()
        assert args.socket_path == "/run/my-mcm.sock"


# ---------------------------------------------------------------------------
# ErrorStatus (0x30) emission
# ---------------------------------------------------------------------------

def _sent_frame_ids(sim: McmSimulator) -> list[int]:
    return [call.args[0].arbitration_id for call in sim._bus.send.call_args_list]


def _sent_frames_by_id(sim: McmSimulator, frame_id: int) -> list[bytes]:
    return [
        bytes(call.args[0].data)
        for call in sim._bus.send.call_args_list
        if call.args[0].arbitration_id == frame_id
    ]


class TestErrorStatusEmission:
    def test_crc_mismatch_emits_error_status(self) -> None:
        sim = _make_sim()
        # Build a valid ControlEnable frame then corrupt the CRC
        frame = _make_can_frame(
            sim._db,
            "ControlEnable",
            {"BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1},
        )
        bad_data = bytearray(frame.data)
        bad_data[7] ^= 0xFF  # flip all CRC bits
        bad_frame = can.Message(
            arbitration_id=frame.arbitration_id,
            data=bytes(bad_data),
            is_extended_id=False,
        )
        sim._handle_can_message(bad_frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) == 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 1  # CRC Error

    def _make_config_frame(bus_address: int, interface_id: int, section_id: int) -> can.Message:
        """Build a raw ConfigGetSetCommand (0x400) frame.

        DBC layout (see mcm_Configuration.dbc):
          byte 0: SubsystemID[7], BusAddress[6:0]
          byte 1: InterfaceID[7:5], SectionID[4:0]
          byte 2: Config_GetSet[7], ParameterID[6:0]
          bytes 3-6: float32 value (all zeros)
          byte 7: CRC
        """
        raw = bytearray(8)
        raw[0] = bus_address & 0x7F
        raw[1] = ((interface_id & 0x07) << 5) | (section_id & 0x1F)
        raw[2] = 0  # GetSet=0, ParameterID=0
        # bytes 3-6 = 0.0 float32
        from mcm_simulator.crc8 import apply_crc8
        apply_crc8(raw)
        return can.Message(arbitration_id=1024, data=bytes(raw), is_extended_id=False)

    def test_invalid_section_id_in_config_emits_error_status(self) -> None:
        """Section ID 18 (0x12) is not in the known VAL_TABLE → Error_Type 2."""
        sim = _make_sim()
        # Section 18 = 0x12 is not in Error.dbc Config_SectionID VAL_TABLE
        frame = TestErrorStatusEmission._make_config_frame(
            bus_address=1, interface_id=0, section_id=18
        )
        sim._handle_can_message(frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) >= 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 2  # Section ID Invalid

    def test_invalid_interface_id_in_config_emits_error_status(self) -> None:
        """Interface ID 7 > max physical interface (6) → Error_Type 3."""
        sim = _make_sim()
        frame = TestErrorStatusEmission._make_config_frame(
            bus_address=1, interface_id=7, section_id=0
        )
        sim._handle_can_message(frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) >= 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 3  # Interface ID Invalid


# ---------------------------------------------------------------------------
# Task 5: Control/Relay rejection in FAIL_HARD + per-subsystem responses
# ---------------------------------------------------------------------------

class TestControlRejectionInFaultState:
    def _put_in_fail_hard(self, sim: McmSimulator) -> None:
        for s in sim._subsystems:
            s.state = SystemState.FAIL_HARD

    def _put_in_human_control(self, sim: McmSimulator) -> None:
        for s in sim._subsystems:
            s.state = SystemState.HUMAN_CONTROL

    def test_control_enable_in_fail_hard_emits_error_status(self) -> None:
        sim = _make_sim()
        self._put_in_fail_hard(sim)
        t_before = [s.last_rx_time for s in sim._subsystems]
        frame = _make_can_frame(
            sim._db, "ControlEnable",
            {"BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1},
        )
        sim._handle_can_message(frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) >= 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 6  # State Error
        # Normal ControlEnableResponse must NOT be emitted
        from mcm_simulator.simulator import _ID_CONTROL_ENABLE_RESP
        assert _sent_frames_by_id(sim, _ID_CONTROL_ENABLE_RESP) == []
        # Watchdog must NOT have been fed
        for s, t in zip(sim._subsystems, t_before):
            assert s.last_rx_time == t

    def test_control_command_in_fail_hard_emits_error_status(self) -> None:
        sim = _make_sim()
        self._put_in_fail_hard(sim)
        frame = _make_can_frame(
            sim._db, "ControlCommand",
            {"BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Count8": 0, "Value": 0.0},
        )
        sim._handle_can_message(frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) >= 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 6
        assert _sent_frames_by_id(sim, _ID_CONTROL_CMD_RESP) == []

    def test_relay_command_in_fail_hard_emits_error_status(self) -> None:
        sim = _make_sim()
        self._put_in_fail_hard(sim)
        frame = _make_can_frame(
            sim._db, "RelayCommand",
            {"BusAddress": 1, "SubsystemID": 0, "Enable": 1},
        )
        sim._handle_can_message(frame)
        error_frames = _sent_frames_by_id(sim, _ID_ERROR_STATUS)
        assert len(error_frames) >= 1
        decoded = sim._db.decode_message(_ID_ERROR_STATUS, error_frames[0], decode_choices=False)
        assert decoded["Error_Type"] == 6
        assert _sent_frames_by_id(sim, _ID_RELAY_CMD_RESP) == []

    def test_control_enable_in_human_control_emits_one_response_per_subsystem(self) -> None:
        """In HUMAN_CONTROL → MCM_CONTROL, one ControlEnableResponse per subsystem."""
        sim = _make_sim()
        self._put_in_human_control(sim)
        frame = _make_can_frame(
            sim._db, "ControlEnable",
            {"BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Enable": 1},
        )
        sim._handle_can_message(frame)
        from mcm_simulator.simulator import _ID_CONTROL_ENABLE_RESP
        resp_frames = _sent_frames_by_id(sim, _ID_CONTROL_ENABLE_RESP)
        assert len(resp_frames) == len(sim._subsystems)

    def test_rejected_frame_does_not_feed_watchdog(self) -> None:
        """A frame rejected due to fault state must not reset the watchdog timer."""
        sim = _make_sim()
        self._put_in_fail_hard(sim)
        original_rx_times = [s.last_rx_time for s in sim._subsystems]
        frame = _make_can_frame(
            sim._db, "ControlCommand",
            {"BusAddress": 1, "SubSystemID": 0, "InterfaceID": 0, "Count8": 0, "Value": 0.0},
        )
        sim._handle_can_message(frame)
        for s, orig in zip(sim._subsystems, original_rx_times):
            assert s.last_rx_time == orig, "Watchdog was fed by rejected frame"

"""MCM CAN simulator entry point.

Speaks the sygnal CAN protocol on a SocketCAN interface (vcan0 or physical CAN)
so sygnal_can_interface_ros2_node treats this process as real MCM hardware.

Dual-subsystem mode: when --subsystem-ids lists two IDs (e.g. 0,1), two
SubsystemState instances run on the same CAN bus. Each has an independent
watchdog, independent fault-clear challenge, and independent state machine.
Both respond to the same ControlEnable (0x60) and ControlCommand (0x160)
frames. Partial failure injection is available via the Unix socket using
the commands "fail-sub0" and "fail-sub1".

State machine: HUMAN_CONTROL(0) -> MCM_CONTROL(1) via ControlEnable Enable=1;
either state -> FAIL_HARD(254) on watchdog timeout or e-stop press.
FAIL_HARD is cleared via CAN fault clear protocol: MCM publishes HeartbeatClearSeed
(0x70) with a random 32-bit seed; VCU responds with HeartbeatClearKey (0x7F) containing
seed XOR 0xA5A5A5A5. On valid key, MCM transitions to HUMAN_CONTROL.
E-stop FAIL_HARD can also be cleared via Unix socket estop-release command.

Uses asyncio for concurrent heartbeat timer, watchdog timer, CAN recv, and
Unix socket server without threading complexity.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import random
import time
from enum import IntEnum
from pathlib import Path

import can
import cantools

from mcm_simulator.crc8 import apply_crc8, generate_crc8

logger = logging.getLogger(__name__)

_DBC_DIR = Path(__file__).parent / "dbc"


class SystemState(IntEnum):
    HUMAN_CONTROL = 0
    MCM_CONTROL = 1
    FAIL_HARD = 254


# CAN frame IDs from sygnal DBC files
_ID_HEARTBEAT = 368  # 0x170 MCM -> UserApplication
_ID_CONTROL_ENABLE = 96  # 0x60  UserApplication -> MCM
_ID_CONTROL_ENABLE_RESP = 97  # 0x61  MCM -> UserApplication
_ID_CONTROL_CMD = 352  # 0x160 UserApplication -> MCM
_ID_CONTROL_CMD_RESP = 353  # 0x161 MCM -> UserApplication
_ID_RELAY_CMD = 176  # 0xB0  UserApplication -> MCM
_ID_RELAY_CMD_RESP = 177  # 0xB1  MCM -> UserApplication
_ID_HEARTBEAT_CLEAR_SEED = 112  # 0x70 MCM -> UserApplication (fault clear challenge)
_ID_HEARTBEAT_CLEAR_KEY = 127  # 0x7F UserApplication -> MCM (fault clear response)

_FAULT_CLEAR_XOR = 0xA5A5A5A5

_USER_APP_IDS = {
    _ID_CONTROL_ENABLE,
    _ID_CONTROL_CMD,
    _ID_RELAY_CMD,
    _ID_HEARTBEAT_CLEAR_KEY,
}


class SubsystemState:
    """Per-subsystem MCM state machine with independent watchdog and fault clear."""

    def __init__(
        self, subsystem_id: int, bus_address: int, watchdog_timeout_s: float
    ) -> None:
        self.subsystem_id = subsystem_id
        self.bus_address = bus_address
        self.watchdog_timeout_s = watchdog_timeout_s
        # MCM always boots into FAIL_HARD — requires fault clear to reach HUMAN_CONTROL.
        # Matches real MCM hardware behavior.
        self.state = SystemState.FAIL_HARD
        self.estop_latched = False
        self.counter: int = 0
        self.last_rx_time: float = time.monotonic()
        self.fault_clear_seed: int = random.randint(0, 0xFFFFFFFF)

    def transition(self, new_state: SystemState, reason: str) -> None:
        if new_state == self.state:
            return
        logger.info(
            "Sub%d state transition: %s -> %s (%s)",
            self.subsystem_id,
            self.state.name,
            new_state.name,
            reason,
        )
        self.state = new_state

    def handle_control_enable(self, decoded: dict) -> None:
        enable = int(decoded.get("Enable", 0))
        if self.estop_latched:
            logger.warning(
                "Sub%d: ControlEnable ignored: e-stop latched in FAIL_HARD",
                self.subsystem_id,
            )
        elif self.state == SystemState.FAIL_HARD:
            logger.warning(
                "Sub%d: ControlEnable ignored: FAIL_HARD state (non-estop)",
                self.subsystem_id,
            )
        elif enable == 1 and self.state == SystemState.HUMAN_CONTROL:
            self.transition(SystemState.MCM_CONTROL, "ControlEnable Enable=1")
        elif enable == 0 and self.state == SystemState.MCM_CONTROL:
            self.transition(SystemState.HUMAN_CONTROL, "ControlEnable Enable=0")

    def check_watchdog(self) -> bool:
        """Returns True if watchdog just fired (transition to FAIL_HARD)."""
        if self.state != SystemState.FAIL_HARD:
            elapsed = time.monotonic() - self.last_rx_time
            if elapsed > self.watchdog_timeout_s:
                logger.error(
                    "Sub%d watchdog timeout after %.0fms — transitioning to FAIL_HARD",
                    self.subsystem_id,
                    elapsed * 1000,
                )
                self.transition(SystemState.FAIL_HARD, "watchdog timeout")
                return True
        return False

    def feed_watchdog(self) -> None:
        self.last_rx_time = time.monotonic()

    def handle_heartbeat_clear_key(self, decoded: dict) -> None:
        if self.state != SystemState.FAIL_HARD:
            return
        if self.fault_clear_seed == 0:
            logger.warning(
                "Sub%d: HeartbeatClearKey received but no seed published",
                self.subsystem_id,
            )
            return
        reset_key = int(decoded.get("ResetKey", 0))
        expected_key = self.fault_clear_seed ^ _FAULT_CLEAR_XOR
        if reset_key != expected_key:
            logger.warning(
                "Sub%d HeartbeatClearKey mismatch: expected 0x%08X, got 0x%08X",
                self.subsystem_id,
                expected_key,
                reset_key,
            )
            return
        logger.info(
            "Sub%d: Fault clear key validated — clearing FAIL_HARD", self.subsystem_id
        )
        self.estop_latched = False
        self.fault_clear_seed = 0
        self.last_rx_time = time.monotonic()
        self.transition(SystemState.HUMAN_CONTROL, "fault clear key validated")

    def trigger_estop(self) -> None:
        self.estop_latched = True
        self.transition(SystemState.FAIL_HARD, "estop-press")

    def release_estop(self) -> None:
        if self.estop_latched:
            self.estop_latched = False
            self.transition(SystemState.HUMAN_CONTROL, "estop-release")
        else:
            logger.warning(
                "Sub%d: estop-release ignored: FAIL_HARD not caused by estop",
                self.subsystem_id,
            )


def _load_dbc() -> cantools.database.Database:
    db = cantools.database.Database()
    for name in ("Heartbeat.dbc", "Control.dbc", "Relay.dbc"):
        db.add_dbc_file(str(_DBC_DIR / name))
    return db


class McmSimulator:
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        subsystem_ids: list[int] = args.subsystem_ids
        watchdog_timeout_s = args.watchdog_timeout_ms / 1000.0
        self._subsystems: list[SubsystemState] = [
            SubsystemState(sid, args.bus_address, watchdog_timeout_s)
            for sid in subsystem_ids
        ]
        self._bus: can.BusABC | None = None
        self._db = _load_dbc()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._watchdog_task: asyncio.Task | None = None
        self._heartbeat_task: asyncio.Task | None = None
        self._socket_task: asyncio.Task | None = None

    @property
    def _primary(self) -> SubsystemState:
        return self._subsystems[0]

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def _handle_control_enable(self, decoded: dict) -> None:
        for sub in self._subsystems:
            sub.handle_control_enable(decoded)

    def _send_control_enable_response(self, decoded: dict) -> None:
        enabled = 1 if self._primary.state == SystemState.MCM_CONTROL else 0
        msg = self._db.get_message_by_name("ControlEnableResponse")
        data = msg.encode(
            {
                "BusAddress": self._args.bus_address,
                "SubSystemID": self._primary.subsystem_id,
                "InterfaceID": int(decoded.get("InterfaceID", 0)),
                "Enable": enabled,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_CONTROL_ENABLE_RESP, frame)

    def _send_control_command_response(self, decoded: dict) -> None:
        msg = self._db.get_message_by_name("ControlCommandResponse")
        data = msg.encode(
            {
                "BusAddress": self._args.bus_address,
                "SubSystemID": self._primary.subsystem_id,
                "InterfaceID": int(decoded.get("InterfaceID", 0)),
                "Count8": int(decoded.get("Count8", 0)),
                "Value": float(decoded.get("Value", 0.0)),
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_CONTROL_CMD_RESP, frame)

    def _send_relay_command_response(self, decoded: dict) -> None:
        msg = self._db.get_message_by_name("RelayCommandResponse")
        data = msg.encode(
            {
                "BusAddress": int(decoded.get("BusAddress", self._args.bus_address)),
                "SubsystemID": self._primary.subsystem_id,
                "Enable": int(decoded.get("Enable", 0)),
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_RELAY_CMD_RESP, frame)

    def _send_frame(self, frame_id: int, data: bytearray) -> None:
        if self._bus is None:
            return
        try:
            self._bus.send(
                can.Message(
                    arbitration_id=frame_id, data=bytes(data), is_extended_id=False
                )
            )
        except can.CanError as exc:
            logger.error("CAN send error (id=0x%03X): %s", frame_id, exc)

    # ------------------------------------------------------------------
    # Heartbeat publisher
    # ------------------------------------------------------------------

    async def _heartbeat_loop(self) -> None:
        interval = 1.0 / self._args.heartbeat_rate_hz
        while True:
            await asyncio.sleep(interval)
            for sub in self._subsystems:
                self._publish_subsystem_heartbeat(sub)

    def _publish_subsystem_heartbeat(self, sub: SubsystemState) -> None:
        sub.counter = (sub.counter + 1) & 0xFFFF
        iface_state = 1 if sub.state == SystemState.MCM_CONTROL else 0
        msg = self._db.get_message_by_name("Heartbeat")
        data = msg.encode(
            {
                "BusAddress": self._args.bus_address,
                "SubsystemID": sub.subsystem_id,
                "SystemState": int(sub.state),
                "Count16": sub.counter,
                "Interface0State": iface_state,
                "Interface1State": iface_state,
                "Interface2State": iface_state,
                "Interface3State": iface_state,
                "Interface4State": iface_state,
                "Interface5State": iface_state,
                "Interface6State": iface_state,
                "OverallInterfaceState": iface_state,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_HEARTBEAT, frame)

    # ------------------------------------------------------------------
    # Fault clear (challenge-response protocol)
    # ------------------------------------------------------------------

    def _publish_subsystem_fault_clear_seed(self, sub: SubsystemState) -> None:
        """Publish HeartbeatClearSeed (0x70) with a random 32-bit seed for one subsystem."""
        sub.fault_clear_seed = random.getrandbits(32) or 1  # never zero
        msg = self._db.get_message_by_name("HeartbeatClearSeed")
        data = msg.encode(
            {
                "BusAddress": self._args.bus_address,
                "SubsystemID": sub.subsystem_id,
                "ResetSeed": sub.fault_clear_seed,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_HEARTBEAT_CLEAR_SEED, frame)

    async def _fault_clear_seed_loop(self) -> None:
        """Publish fault clear seeds every 500ms while any subsystem is in FAIL_HARD."""
        while True:
            for sub in self._subsystems:
                if sub.state == SystemState.FAIL_HARD:
                    self._publish_subsystem_fault_clear_seed(sub)
            await asyncio.sleep(0.5)

    def _handle_heartbeat_clear_key(self, decoded: dict) -> None:
        """Route HeartbeatClearKey to the subsystem currently in FAIL_HARD."""
        for sub in self._subsystems:
            sub.handle_heartbeat_clear_key(decoded)

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    async def _watchdog_loop(self) -> None:
        # Check every 50ms; trigger FAIL_HARD if gap exceeds watchdog_timeout_ms
        check_interval = 0.050
        while True:
            await asyncio.sleep(check_interval)
            for sub in self._subsystems:
                sub.check_watchdog()

    # ------------------------------------------------------------------
    # CAN receive
    # ------------------------------------------------------------------

    async def _can_receive_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            try:
                msg = await loop.run_in_executor(None, self._bus.recv, 0.1)
            except Exception as exc:
                logger.error("CAN recv error: %s", exc)
                await asyncio.sleep(0.1)
                continue
            if msg is None:
                continue
            self._handle_can_message(msg)

    def _handle_can_message(self, msg: can.Message) -> None:
        if msg.arbitration_id not in _USER_APP_IDS:
            return
        frame = bytearray(msg.data)
        if len(frame) < 8:
            frame.extend([0] * (8 - len(frame)))
        if frame[7] != generate_crc8(frame):
            logger.warning("Dropped frame 0x%03X: CRC mismatch", msg.arbitration_id)
            return
        try:
            decoded = self._db.decode_message(msg.arbitration_id, bytes(frame))
        except Exception as exc:
            logger.warning("Failed to decode frame 0x%03X: %s", msg.arbitration_id, exc)
            return
        if int(decoded.get("BusAddress", 0)) != self._args.bus_address:
            return
        if msg.arbitration_id != _ID_HEARTBEAT_CLEAR_KEY:
            # Fault clear key does NOT reset the watchdog — only control commands do.
            for sub in self._subsystems:
                sub.feed_watchdog()
        if msg.arbitration_id == _ID_CONTROL_ENABLE:
            self._handle_control_enable(decoded)
            self._send_control_enable_response(decoded)
        elif msg.arbitration_id == _ID_CONTROL_CMD:
            self._send_control_command_response(decoded)
        elif msg.arbitration_id == _ID_RELAY_CMD:
            self._send_relay_command_response(decoded)
        elif msg.arbitration_id == _ID_HEARTBEAT_CLEAR_KEY:
            self._handle_heartbeat_clear_key(decoded)

    # ------------------------------------------------------------------
    # Unix socket e-stop server
    # ------------------------------------------------------------------

    async def _socket_server(self) -> None:
        path = self._args.socket_path
        if os.path.exists(path):
            os.unlink(path)
        server = await asyncio.start_unix_server(self._handle_socket_client, path=path)
        async with server:
            await server.serve_forever()

    async def _handle_socket_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        try:
            data = await reader.readline()
            cmd = data.decode().strip()
            if cmd == "estop-press":
                logger.error("E-stop PRESSED via Unix socket — latching FAIL_HARD")
                for sub in self._subsystems:
                    sub.trigger_estop()
                writer.write(b"ok\n")
            elif cmd == "estop-release":
                logger.info("E-stop RELEASED via Unix socket")
                for sub in self._subsystems:
                    sub.release_estop()
                writer.write(b"ok\n")
            elif cmd == "fail-sub0":
                matching = [s for s in self._subsystems if s.subsystem_id == 0]
                if matching:
                    logger.error(
                        "Injecting partial failure: sub0 FAIL_HARD via Unix socket"
                    )
                    matching[0].trigger_estop()
                    writer.write(b"ok\n")
                else:
                    writer.write(b"sub0 not configured\n")
            elif cmd == "fail-sub1":
                matching = [s for s in self._subsystems if s.subsystem_id == 1]
                if matching:
                    logger.error(
                        "Injecting partial failure: sub1 FAIL_HARD via Unix socket"
                    )
                    matching[0].trigger_estop()
                    writer.write(b"ok\n")
                else:
                    writer.write(b"sub1 not configured\n")
            elif cmd == "recover-sub0":
                matching = [s for s in self._subsystems if s.subsystem_id == 0]
                if matching:
                    logger.info("Recovering sub0 via Unix socket")
                    matching[0].release_estop()
                    writer.write(b"ok\n")
                else:
                    writer.write(b"sub0 not configured\n")
            elif cmd == "recover-sub1":
                matching = [s for s in self._subsystems if s.subsystem_id == 1]
                if matching:
                    logger.info("Recovering sub1 via Unix socket")
                    matching[0].release_estop()
                    writer.write(b"ok\n")
                else:
                    writer.write(b"sub1 not configured\n")
            elif cmd == "query":
                import json as _json

                states = {str(s.subsystem_id): s.state.name for s in self._subsystems}
                writer.write((_json.dumps(states) + "\n").encode())
            else:
                logger.warning("Unknown socket command: %r", cmd)
                writer.write(b"unknown command\n")
            await writer.drain()
        finally:
            writer.close()

    # ------------------------------------------------------------------
    # CAN bus lifecycle
    # ------------------------------------------------------------------

    def _open_bus(self) -> can.BusABC | None:
        retry_delay = 1.0
        while True:
            try:
                bus = can.interface.Bus(
                    channel=self._args.can_interface,
                    interface="socketcan",
                )
                logger.info("CAN bus opened: %s", self._args.can_interface)
                return bus
            except (OSError, can.CanError) as exc:
                logger.error(
                    "CAN bus unavailable (%s): %s — retrying in %.0fs",
                    self._args.can_interface,
                    exc,
                    retry_delay,
                )
                time.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 30.0)

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    async def run(self) -> None:
        logger.info(
            "Starting MCM simulator with subsystems %s",
            [s.subsystem_id for s in self._subsystems],
        )
        self._bus = self._open_bus()
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._heartbeat_loop())
            tg.create_task(self._watchdog_loop())
            tg.create_task(self._fault_clear_seed_loop())
            tg.create_task(self._can_receive_loop())
            tg.create_task(self._socket_server())


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulated MCM CAN node (sygnal protocol)"
    )
    parser.add_argument("--can-interface", default="vcan0")
    # 2000ms default: Python asyncio event loop scheduling adds hundreds of ms
    # jitter that real MCM firmware doesn't have. Real hardware uses 200ms.
    parser.add_argument("--watchdog-timeout-ms", type=int, default=2000)
    parser.add_argument("--heartbeat-rate-hz", type=float, default=15.0)
    parser.add_argument("--bus-address", type=int, default=1)
    parser.add_argument(
        "--subsystem-ids",
        type=lambda s: [int(x) for x in s.split(",")],
        default=[0, 1],
        help="Comma-separated subsystem IDs to simulate (default: 0,1)",
    )
    parser.add_argument(
        "--socket-path",
        default=None,
        help="Unix socket path for estop/query commands (default: /tmp/simulated-mcm-{bus_address}.sock)",
    )
    args = parser.parse_args()
    if args.socket_path is None:
        args.socket_path = f"/tmp/simulated-mcm-{args.bus_address}.sock"
    return args


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    args = _parse_args()
    asyncio.run(McmSimulator(args).run())


if __name__ == "__main__":
    main()

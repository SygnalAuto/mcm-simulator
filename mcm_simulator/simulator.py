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
from dataclasses import dataclass, field
from collections.abc import Callable
from enum import IntEnum
from pathlib import Path

import can
import cantools

from mcm_simulator.config_store import ConfigStore
from mcm_simulator.crc8 import apply_crc8, generate_crc8
from mcm_simulator.device_config import DeviceSpec, parse_devices_spec

logger = logging.getLogger(__name__)

_DBC_DIR = Path(__file__).parent / "dbc"


class SystemState(IntEnum):
    HUMAN_CONTROL = 0
    MCM_CONTROL = 1
    FAIL_OPERATIONAL_1 = 241   # 0xF1 — enum only; sim never enters organically
    FAIL_OPERATIONAL_2 = 242   # 0xF2 — enum only; sim never enters organically
    HUMAN_OVERRIDE = 253       # 0xFD — enum only; sim has no analog signal path
    FAIL_HARD = 254


# CAN frame IDs from sygnal DBC files
_ID_HEARTBEAT = 368  # 0x170 MCM -> UserApplication
_ID_CONTROL_ENABLE = 96  # 0x60  UserApplication -> MCM
_ID_CONTROL_ENABLE_RESP = 97  # 0x61  MCM -> UserApplication
_ID_CONTROL_CMD = 352  # 0x160 UserApplication -> MCM
_ID_CONTROL_CMD_RESP = 353  # 0x161 MCM -> UserApplication
_ID_RELAY_CMD = 176  # 0xB0  UserApplication -> MCM
_ID_RELAY_CMD_RESP = 177  # 0xB1  MCM -> UserApplication
_ID_FAULT_STATE = 32             # 0x20  MCM -> UserApplication (current fault state)
_ID_FAULT_INCREMENT = 33         # 0x21  MCM -> UserApplication (fault occurrence)
_ID_ERROR_STATUS = 48            # 0x30  MCM -> UserApplication (config error)
_ID_HEARTBEAT_CLEAR_SEED = 112   # 0x70  MCM -> UserApplication (fault clear challenge)
_ID_HEARTBEAT_CLEAR_KEY = 127    # 0x7F  UserApplication -> MCM (fault clear response)
_ID_FAULT_LIST = 544             # 0x220 MCM -> UserApplication (fault list)
_ID_FAULT_ROOT_CAUSE = 545       # 0x221 MCM -> UserApplication (fault root cause)
_ID_REPORT_DATA = 1792           # 0x700 MCM -> UserApplication (telemetry signal)

# Configuration protocol CAN IDs (same for MCM, CB, IO — filtered by BusAddress in frame)
_ID_CONFIG_GETSET_CMD = 1024   # 0x400 UserApplication -> Device
_ID_CONFIG_GETSET_RESP = 1025  # 0x401 Device -> UserApplication

# Identify protocol CAN IDs (same for MCM, CB, IO — all respond to the same broadcast)
_ID_IDENTIFY_CMD = 1536          # 0x600 UserApplication -> all devices (0-byte broadcast)
_ID_IDENTIFY_RESP_MAIN = 1537    # 0x601 Device -> UserApplication (ProductID, serial, boot state)
_ID_IDENTIFY_RESP_APP_VERSION = 1538  # 0x602 Device -> UserApplication (app firmware version)
_ID_IDENTIFY_RESP_BL_VERSION = 1539   # 0x603 Device -> UserApplication (bootloader version)

_FAULT_CLEAR_XOR = 0xA5A5A5A5

_USER_APP_IDS = {
    _ID_CONTROL_ENABLE,
    _ID_CONTROL_CMD,
    _ID_RELAY_CMD,
    _ID_HEARTBEAT_CLEAR_KEY,
    _ID_IDENTIFY_CMD,
    _ID_CONFIG_GETSET_CMD,
}

# Maps device product_type → Configuration.dbc filename prefix
_CONFIG_DBC_BY_TYPE: dict[str, str] = {
    "mcm": "mcm_Configuration.dbc",
    "cb": "cb_Configuration.dbc",
    "io": "io_Configuration.dbc",
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
        self.fault_count: int = 0
        self.fault_cause: int = 0
        self.on_fault_increment: Callable[[SubsystemState], None] | None = None

    # Official MCM fault source IDs (from Sygnal firmware documentation)
    _FAULT_CAUSE_ESTOP = 0x01       # Emergency Stop
    _FAULT_CAUSE_EXTERNAL = 0x02    # External Fault
    _FAULT_CAUSE_WATCHDOG = 0x30    # Customer Application Heartbeat Timeout

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
        if new_state == SystemState.FAIL_HARD:
            self.fault_count += 1
            if "watchdog" in reason.lower():
                self.fault_cause = self._FAULT_CAUSE_WATCHDOG
            elif "estop" in reason.lower() or "e-stop" in reason.lower():
                self.fault_cause = self._FAULT_CAUSE_ESTOP
            else:
                self.fault_cause = 255  # unknown
            if self.on_fault_increment is not None:
                self.on_fault_increment(self)
        elif self.state == SystemState.FAIL_HARD:
            # Clearing fault — reset cause
            self.fault_cause = 0
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


def _is_fault_state(state: SystemState) -> bool:
    """Return True for states in which the MCM is actively faulted.

    Used to gate FaultState (0x20), FaultList (0x220), FaultRootCause (0x221),
    and HeartbeatClearSeed (0x70) emission — those frames are only sent while
    the device is in a fault condition per sygnal_docs v2.1.1.
    """
    return state in (
        SystemState.FAIL_HARD,
        SystemState.FAIL_OPERATIONAL_1,
        SystemState.FAIL_OPERATIONAL_2,
        SystemState.HUMAN_OVERRIDE,
    )


def _load_dbc() -> cantools.database.Database:
    db = cantools.database.Database()
    for name in (
        "Heartbeat.dbc", "Control.dbc", "Relay.dbc", "Identify.dbc",
        "Fault.dbc", "Report.dbc", "Error.dbc",
    ):
        db.add_dbc_file(str(_DBC_DIR / name))
    return db


def _load_config_db(product_type: str) -> cantools.database.Database | None:
    """Load the device-type-specific Configuration.dbc.  Returns None if not found."""
    dbc_name = _CONFIG_DBC_BY_TYPE.get(product_type)
    if dbc_name is None:
        return None
    dbc_path = _DBC_DIR / dbc_name
    if not dbc_path.exists():
        logger.warning("Configuration DBC not found: %s", dbc_path)
        return None
    db = cantools.database.Database()
    db.add_dbc_file(str(dbc_path))
    return db


@dataclass
class _DeviceNode:
    """A single simulated Sygnal device: one bus address, N subsystems, one identity."""

    spec: DeviceSpec
    subsystems: list[SubsystemState] = field(default_factory=list)
    config_db: cantools.database.Database | None = None


class McmSimulator:
    def __init__(self, args: argparse.Namespace) -> None:
        self._args = args
        watchdog_timeout_s = args.watchdog_timeout_ms / 1000.0

        # Build device nodes.  Prefer explicit --devices list; fall back to legacy
        # --bus-address + --subsystem-ids for single-device backwards compatibility.
        devices: list[DeviceSpec] | None = getattr(args, "devices", None)
        if devices:
            self._device_nodes: list[_DeviceNode] = [
                _DeviceNode(
                    spec=spec,
                    subsystems=[
                        SubsystemState(sid, spec.bus_address, watchdog_timeout_s)
                        for sid in spec.subsystem_ids
                    ],
                )
                for spec in devices
            ]
        else:
            # Legacy single-device mode
            bus_address: int = args.bus_address
            subsystem_ids: list[int] = args.subsystem_ids
            legacy_spec = DeviceSpec(
                bus_address=bus_address,
                product_type="mcm",
                product_id=1,
            )
            legacy_subs = [
                SubsystemState(sid, bus_address, watchdog_timeout_s)
                for sid in subsystem_ids
            ]
            self._device_nodes = [_DeviceNode(spec=legacy_spec, subsystems=legacy_subs)]

        # Load per-device-type config DBs
        for node in self._device_nodes:
            node.config_db = _load_config_db(node.spec.product_type)

        # _subsystems stays as the primary device's subsystems for backwards compat.
        # Existing control/watchdog/socket logic uses this.
        self._subsystems: list[SubsystemState] = self._device_nodes[0].subsystems

        self._config_store = ConfigStore()
        self._bus: can.BusABC | None = None
        self._db = _load_dbc()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._watchdog_task: asyncio.Task | None = None
        self._heartbeat_task: asyncio.Task | None = None
        self._socket_task: asyncio.Task | None = None

        # Wire fault increment callback so transitions publish FaultIncrement frames
        for node in self._device_nodes:
            for sub in node.subsystems:
                sub.on_fault_increment = self._publish_fault_increment

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
        msg = self._db.get_message_by_name("ControlEnableResponse")
        for sub in self._subsystems:
            enabled = 1 if sub.state == SystemState.MCM_CONTROL else 0
            data = msg.encode(
                {
                    "BusAddress": self._args.bus_address,
                    "SubSystemID": sub.subsystem_id,
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
        for sub in self._subsystems:
            data = msg.encode(
                {
                    "BusAddress": self._args.bus_address,
                    "SubSystemID": sub.subsystem_id,
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
        for sub in self._subsystems:
            data = msg.encode(
                {
                    "BusAddress": int(decoded.get("BusAddress", self._args.bus_address)),
                    "SubsystemID": sub.subsystem_id,
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
            self._publish_all_heartbeats()

    def _publish_all_heartbeats(self) -> None:
        """Publish heartbeats and (when faulted) fault state for every subsystem."""
        for node in self._device_nodes:
            for sub in node.subsystems:
                self._publish_subsystem_heartbeat(sub)
                if _is_fault_state(sub.state):
                    self._publish_subsystem_fault_state(sub)

    def _publish_subsystem_heartbeat(self, sub: SubsystemState) -> None:
        sub.counter = (sub.counter + 1) & 0xFFFF
        iface_state = 1 if sub.state == SystemState.MCM_CONTROL else 0
        msg = self._db.get_message_by_name("Heartbeat")
        data = msg.encode(
            {
                "BusAddress": sub.bus_address,  # Use sub's own address (multi-device safe)
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
    # Diagnostic frame publishers
    # ------------------------------------------------------------------

    def _publish_subsystem_fault_state(self, sub: SubsystemState) -> None:
        """Publish FaultState (0x20) for one subsystem."""
        msg = self._db.get_message_by_name("FaultState")
        data = msg.encode(
            {
                "BusAddress": sub.bus_address,
                "SubsystemID": sub.subsystem_id,
                "FaultState": int(sub.state),
                "FaultCause": sub.fault_cause,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_FAULT_STATE, frame)

    def _publish_fault_increment(self, sub: SubsystemState) -> None:
        """Publish FaultIncrement (0x21) — called once per fault transition."""
        msg = self._db.get_message_by_name("FaultIncrement")
        data = msg.encode(
            {
                "BusAddress": sub.bus_address,
                "SubsystemID": sub.subsystem_id,
                "FaultType": sub.fault_cause,
                "FaultCount": sub.fault_count,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_FAULT_INCREMENT, frame)

    def _publish_report_signal(
        self,
        bus_address: int,
        subsystem_id: int,
        interface_id: int,
        signal_id: int,
        value: float,
    ) -> None:
        """Publish a single ReportData (0x700) frame."""
        msg = self._db.get_message_by_name("ReportData")
        data = msg.encode(
            {
                "BusAddress": bus_address,
                "SubsystemID": subsystem_id,
                "InterfaceID": interface_id,
                "Report_SignalID": signal_id,
                "Report_Value_FLOAT32": value,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_REPORT_DATA, frame)

    def _publish_error_status(
        self,
        bus_address: int,
        subsystem_id: int,
        error_type: int,
        section_id: int,
        interface_id: int,
        can_id: int,
    ) -> None:
        """Publish ErrorStatus (0x30) frame."""
        msg = self._db.get_message_by_name("ErrorStatus")
        data = msg.encode(
            {
                "BusAddress": bus_address,
                "SubsystemID": subsystem_id,
                "Error_Type": error_type,
                "Config_SectionID": section_id,
                "Config_InterfaceID": interface_id,
                "Error_CANID": can_id,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_ERROR_STATUS, frame)

    # ------------------------------------------------------------------
    # Report data loop (synthetic telemetry)
    # ------------------------------------------------------------------

    # Report_SignalID values to broadcast — all from sygnal_docs v2.1.1 §Report.
    # TODO: per-signal rate config via ConfigGetSet Section 31 (Report) is deferred;
    #       this always-on broadcast diverges from the docs' opt-in model (see audit).
    _REPORT_SIGNALS: list[tuple[int, str]] = [
        (0x80, "Signal Input"),
        (0x81, "Signal Output Actual"),
        (0x83, "Signal Output Target"),
        (0x50, "Feedback Raw"),
        (0x52, "Closed Loop Setpoint"),
        (0x40, "Override State"),
    ]

    async def _report_data_loop(self) -> None:
        """Publish synthetic ReportData signals at the configured rate."""
        interval = 1.0 / self._args.report_rate_hz
        tick = 0
        while True:
            await asyncio.sleep(interval)
            tick += 1
            for node in self._device_nodes:
                for sub in node.subsystems:
                    for signal_id, _ in self._REPORT_SIGNALS:
                        value = self._synthetic_value(signal_id, tick)
                        self._publish_report_signal(
                            bus_address=sub.bus_address,
                            subsystem_id=sub.subsystem_id,
                            interface_id=0,
                            signal_id=signal_id,
                            value=value,
                        )

    @staticmethod
    def _synthetic_value(signal_id: int, tick: int) -> float:
        """Generate a plausible synthetic value for a given signal.

        Values are keyed to documented Signal IDs (mcm-can-api-guide.md §Report).
        ADC scale is 0–4096 (12-bit) per signal-reporting.md.
        """
        import math  # local import to avoid top-level dependency

        if signal_id in (0x80, 0x81, 0x83):
            # Signal Input / Output Actual / Output Target: sine over 12-bit ADC range
            return 2048.0 + 2048.0 * math.sin(tick * 0.05 + signal_id * 0.3)
        if signal_id == 0x50:
            # Feedback Raw: faster sine centred at midscale
            return 2048.0 + 2048.0 * math.sin(tick * 0.1)
        if signal_id == 0x52:
            # Closed Loop Setpoint: step between 0.0 and 1.0
            return 1.0 if (tick // 50) % 2 == 0 else 0.0
        if signal_id == 0x40:
            # Override State: always Normal (0) — sim has no HUO detection
            return 0.0
        return 0.0

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

    def _publish_fault_list(self, sub: SubsystemState) -> None:
        """Publish FaultList (0x220) for one subsystem.

        Emits one frame for the current fault_cause if fault_count > 0.
        Real MCM emits one frame per non-zero cause; sim tracks a single
        cause so at most one frame per tick. See docs compliance audit.
        """
        if sub.fault_count == 0:
            return
        msg = self._db.get_message_by_name("FaultList")
        data = msg.encode(
            {
                "BusAddress": sub.bus_address,
                "SubsystemID": sub.subsystem_id,
                "FaultType": sub.fault_cause,
                "FaultCount": sub.fault_count,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_FAULT_LIST, frame)

    def _publish_fault_root_cause(self, sub: SubsystemState) -> None:
        """Publish FaultRootCause (0x221) for one subsystem.

        Fills FailHardCause from sub.fault_cause; FailOp1/2 are always 0
        since the sim never enters those states organically.
        """
        fail_hard_cause = sub.fault_cause if sub.state == SystemState.FAIL_HARD else 0
        msg = self._db.get_message_by_name("FaultRootCause")
        data = msg.encode(
            {
                "BusAddress": sub.bus_address,
                "SubsystemID": sub.subsystem_id,
                "FailOp1Cause": 0,
                "FailOp2Cause": 0,
                "FailHardCause": fail_hard_cause,
                "CRC": 0,
            }
        )
        frame = bytearray(data)
        apply_crc8(frame)
        self._send_frame(_ID_FAULT_ROOT_CAUSE, frame)

    async def _fault_clear_seed_loop(self) -> None:
        """Publish fault-state diagnostic frames at the seed rate (500ms).

        Emits HeartbeatClearSeed, FaultList, and FaultRootCause for each
        subsystem that is in an active fault state.
        """
        while True:
            for sub in self._subsystems:
                if _is_fault_state(sub.state):
                    self._publish_subsystem_fault_clear_seed(sub)
                    self._publish_fault_list(sub)
                    self._publish_fault_root_cause(sub)
            await asyncio.sleep(0.5)

    def _handle_heartbeat_clear_key(self, decoded: dict) -> None:
        """Route HeartbeatClearKey to the subsystem currently in FAIL_HARD."""
        for sub in self._subsystems:
            sub.handle_heartbeat_clear_key(decoded)

    # ------------------------------------------------------------------
    # ConfigGetSet protocol
    # ------------------------------------------------------------------

    def _handle_config_command(self, msg: can.Message) -> None:
        """Handle ConfigGetSetCommand (0x400): look up device by BusAddress, store or
        retrieve the parameter value, and send ConfigGetSetResponse (0x401) for each
        subsystem of the matching device.

        Uses decode_choices=False so all values are raw integers (no NamedSignalValue).
        """
        # Find the device node matching the BusAddress in the frame.
        # We peek at raw byte 0 (bits 0-6) for BusAddress to avoid decode overhead.
        raw = bytearray(msg.data)
        if len(raw) < 8:
            raw.extend([0] * (8 - len(raw)))
        bus_addr_raw = raw[0] & 0x7F  # bits 0-6

        node: _DeviceNode | None = None
        for n in self._device_nodes:
            if n.spec.bus_address == bus_addr_raw:
                node = n
                break
        if node is None or node.config_db is None:
            return

        # Peek section_id / interface_id from raw bytes (before decode) so we can
        # validate and emit ErrorStatus even when the multiplexed DBC decode fails.
        # Byte 1: InterfaceID[7:5], SectionID[4:0]
        section_id_raw: int = raw[1] & 0x1F
        iface_id_raw: int = (raw[1] >> 5) & 0x07
        sub_id_raw: int = (raw[0] >> 7) & 0x01

        try:
            cmd_msg = node.config_db.get_message_by_name("ConfigGetSetCommand")
            decoded = node.config_db.decode_message(
                cmd_msg.frame_id, bytes(raw), decode_choices=False
            )
        except Exception as exc:
            # Decode fails for unknown/invalid section IDs (mux ID out of range).
            logger.warning("Failed to decode ConfigGetSetCommand: %s", exc)
            self._publish_error_status(
                bus_address=bus_addr_raw, subsystem_id=sub_id_raw,
                error_type=2, section_id=section_id_raw, interface_id=iface_id_raw,
                can_id=_ID_CONFIG_GETSET_CMD,
            )
            return

        section_id: int = int(decoded.get("Config_SectionID", 0))
        iface_id: int = int(decoded.get("Config_InterfaceID", 0))
        is_set: bool = bool(int(decoded.get("Config_GetSet", 0)))
        sub_id: int = int(decoded.get("SubsystemID", 0))

        # Physical interfaces are 0-6 (7 virtual interfaces per overview.md)
        if iface_id > 6:
            logger.warning(
                "ConfigGetSet: interface ID %d > 6 — emitting ErrorStatus", iface_id
            )
            self._publish_error_status(
                bus_address=bus_addr_raw, subsystem_id=sub_id,
                error_type=3, section_id=section_id, interface_id=iface_id,
                can_id=_ID_CONFIG_GETSET_CMD,
            )
            return

        # Find the active Config_ParameterID_* and Config_Value_* signals
        param_key: str | None = None
        param_id: int = 0
        value_key: str | None = None
        current_value: float = 0.0

        for k, v in decoded.items():
            if k.startswith("Config_ParameterID_") and param_key is None:
                param_key = k
                param_id = int(v)
            elif k.startswith("Config_Value_") and value_key is None:
                value_key = k
                current_value = float(v)

        if param_key is None or value_key is None:
            logger.debug("No param/value signals in decoded ConfigGetSetCommand")
            return

        if is_set:
            # Store for ALL subsystems under this device (config is device-wide,
            # not per-subsystem, but we key by sub_id as received so each sub
            # can independently track its own store if needed)
            self._config_store.set(
                bus_addr_raw, int(decoded.get("SubsystemID", 0)),
                iface_id, section_id, param_id, current_value
            )
            stored_value = current_value
        else:
            # Retrieve the stored value (returns 0.0 if never set)
            stored_value = self._config_store.get(
                bus_addr_raw, int(decoded.get("SubsystemID", 0)),
                iface_id, section_id, param_id
            )

        # Send ConfigGetSetResponse for each subsystem of this device
        try:
            resp_msg = node.config_db.get_message_by_name("ConfigGetSetResponse")
            for sub in node.subsystems:
                resp_signals = dict(decoded)
                resp_signals["SubsystemID"] = sub.subsystem_id
                resp_signals[value_key] = stored_value
                resp_signals["CRC"] = 0
                resp_raw = bytearray(resp_msg.encode(resp_signals))
                apply_crc8(resp_raw)
                self._send_frame(_ID_CONFIG_GETSET_RESP, resp_raw)
        except Exception as exc:
            logger.warning("Failed to encode ConfigGetSetResponse: %s", exc)

    # ------------------------------------------------------------------
    # Identify protocol
    # ------------------------------------------------------------------

    def _handle_identify_command(self) -> None:
        """Respond to IdentifyCommand (0x600) with identity frames for all devices.

        Each device responds for each of its subsystems with three frames:
          IdentifyResponseMain (0x601) — ProductID, serial, boot state
          IdentifyResponseAppVersion (0x602) — app firmware version
          IdentifyResponseBLVersion (0x603) — bootloader version

        IdentifyCommand is a 0-byte broadcast; no CRC or bus address filtering.
        """
        resp_main = self._db.get_message_by_name("IdentifyResponseMain")
        resp_app = self._db.get_message_by_name("IdentifyResponseAppVersion")
        resp_bl = self._db.get_message_by_name("IdentifyResponseBLVersion")

        for node in self._device_nodes:
            spec = node.spec
            for sub in node.subsystems:
                # IdentifyResponseMain — no CRC field in this DBC message
                main_data = resp_main.encode(
                    {
                        "BusAddress": spec.bus_address,
                        "SubsystemID": sub.subsystem_id,
                        "ModuleBootState": 1,  # 1 = running (not in bootloader)
                        "ProductID": spec.product_id,
                        "ModuleSerialNumber": spec.serial_number,
                    }
                )
                self._send_frame(_ID_IDENTIFY_RESP_MAIN, bytearray(main_data))

                # IdentifyResponseAppVersion — no CRC field
                app_data = resp_app.encode(
                    {
                        "BusAddress": spec.bus_address,
                        "SubsystemID": sub.subsystem_id,
                        "SoftwareVersionMajor": spec.fw_major,
                        "SoftwareVersionMinor": spec.fw_minor,
                        "SoftwareVersionPatch": spec.fw_patch,
                    }
                )
                self._send_frame(_ID_IDENTIFY_RESP_APP_VERSION, bytearray(app_data))

                # IdentifyResponseBLVersion — no CRC field
                bl_data = resp_bl.encode(
                    {
                        "BusAddress": spec.bus_address,
                        "SubsystemID": sub.subsystem_id,
                        "SoftwareVersionMajor": spec.bl_major,
                        "SoftwareVersionMinor": spec.bl_minor,
                        "SoftwareVersionPatch": spec.bl_patch,
                    }
                )
                self._send_frame(_ID_IDENTIFY_RESP_BL_VERSION, bytearray(bl_data))

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    async def _watchdog_loop(self) -> None:
        # 10ms check interval: leaves ~170ms slack under the 200ms default watchdog.
        check_interval = 0.010
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
                msg = await loop.run_in_executor(None, self._bus.recv, 0.02)
            except Exception as exc:
                logger.error("CAN recv error: %s", exc)
                await asyncio.sleep(0.02)
                continue
            if msg is None:
                continue
            self._handle_can_message(msg)

    def _handle_can_message(self, msg: can.Message) -> None:
        if msg.arbitration_id not in _USER_APP_IDS:
            return

        # IdentifyCommand (0x600) is a 0-byte broadcast — no CRC, no bus address.
        # Respond immediately for all device nodes and return.
        if msg.arbitration_id == _ID_IDENTIFY_CMD:
            self._handle_identify_command()
            return

        frame = bytearray(msg.data)
        if len(frame) < 8:
            frame.extend([0] * (8 - len(frame)))
        if frame[7] != generate_crc8(frame):
            logger.warning("Dropped frame 0x%03X: CRC mismatch", msg.arbitration_id)
            # Emit ErrorStatus(Error_Type=1 CRC Error) per docs. SubsystemID unknown before
            # decode, so use 0 and primary device's bus_address.
            self._publish_error_status(
                bus_address=self._args.bus_address,
                subsystem_id=0,
                error_type=1,
                section_id=0,
                interface_id=0,
                can_id=msg.arbitration_id,
            )
            return

        # ConfigGetSetCommand uses device-type-specific DBC — address routing is
        # handled inside _handle_config_command, so dispatch before the main decode.
        if msg.arbitration_id == _ID_CONFIG_GETSET_CMD:
            self._handle_config_command(msg)
            return

        try:
            decoded = self._db.decode_message(msg.arbitration_id, bytes(frame))
        except Exception as exc:
            logger.warning("Failed to decode frame 0x%03X: %s", msg.arbitration_id, exc)
            return
        if int(decoded.get("BusAddress", 0)) != self._args.bus_address:
            return

        # For control/relay frames, check fault state BEFORE feeding the watchdog.
        # If any subsystem is faulted: emit ErrorStatus(State Error) per sub,
        # skip watchdog feed, and skip the normal response handler.
        # Docs: "Control messages are only acted upon if the device isn't in an
        # override OR fault state." (mcm-can-api-guide.md)
        _control_ids = {_ID_CONTROL_ENABLE, _ID_CONTROL_CMD, _ID_RELAY_CMD}
        if msg.arbitration_id in _control_ids:
            faulted_subs = [s for s in self._subsystems if _is_fault_state(s.state)]
            if faulted_subs:
                for sub in faulted_subs:
                    self._publish_error_status(
                        bus_address=self._args.bus_address,
                        subsystem_id=sub.subsystem_id,
                        error_type=6,  # State Error
                        section_id=0,
                        interface_id=int(decoded.get("InterfaceID", 0)),
                        can_id=msg.arbitration_id,
                    )
                return  # do not feed watchdog, do not send normal response

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
    # CI hook helpers (called from _handle_socket_client)
    # ------------------------------------------------------------------

    # Maps socket command state names → SystemState values
    _CI_STATE_NAMES: dict[str, SystemState] = {
        "HUMAN_CONTROL": SystemState.HUMAN_CONTROL,
        "MCM_CONTROL": SystemState.MCM_CONTROL,
        "FAIL_HARD": SystemState.FAIL_HARD,
        "FAIL_OP_1": SystemState.FAIL_OPERATIONAL_1,
        "FAIL_OP_2": SystemState.FAIL_OPERATIONAL_2,
        "HUMAN_OVERRIDE": SystemState.HUMAN_OVERRIDE,
    }

    def _ci_find_sub(self, sub_id: int) -> SubsystemState | None:
        for s in self._subsystems:
            if s.subsystem_id == sub_id:
                return s
        return None

    def _ci_set_state(self, args: str) -> str:
        """Handle 'set-state <sub-id> <state-name>'. Returns response line."""
        parts = args.split(None, 1)
        if len(parts) != 2:
            return "error: usage: set-state <sub-id> <state-name>\n"
        try:
            sub_id = int(parts[0])
        except ValueError:
            return f"error: invalid sub-id {parts[0]!r}\n"
        state_name = parts[1].strip()
        if state_name not in self._CI_STATE_NAMES:
            return f"error: unknown state {state_name!r}\n"
        sub = self._ci_find_sub(sub_id)
        if sub is None:
            return f"error: subsystem {sub_id} not configured\n"
        new_state = self._CI_STATE_NAMES[state_name]
        sub.transition(new_state, f"CI set-state {state_name}")
        # Feed the watchdog so the forced state persists; without this the 200ms
        # default watchdog would immediately trip back to FAIL_HARD if last_rx_time
        # is stale (e.g. right after boot).
        if not _is_fault_state(new_state):
            sub.feed_watchdog()
        logger.info("CI: set sub%d state → %s", sub_id, state_name)
        return "ok\n"

    def _ci_inject_fault(self, args: str) -> str:
        """Handle 'inject-fault <sub-id> <fault-cause>'. Increments counter, emits FaultIncrement."""
        parts = args.split(None, 1)
        if len(parts) != 2:
            return "error: usage: inject-fault <sub-id> <fault-cause>\n"
        try:
            sub_id = int(parts[0])
            cause_str = parts[1].strip()
            fault_cause = int(cause_str, 0)  # hex or decimal
        except ValueError:
            return f"error: invalid args {args!r}\n"
        sub = self._ci_find_sub(sub_id)
        if sub is None:
            return f"error: subsystem {sub_id} not configured\n"
        sub.fault_cause = fault_cause
        sub.fault_count += 1
        self._publish_fault_increment(sub)
        logger.info("CI: inject fault 0x%02X on sub%d (count=%d)", fault_cause, sub_id, sub.fault_count)
        return "ok\n"

    def _ci_emit_error(self, args: str) -> str:
        """Handle 'emit-error <sub-id> <error-type> <can-id>'. Emits one ErrorStatus frame."""
        parts = args.split(None, 2)
        if len(parts) != 3:
            return "error: usage: emit-error <sub-id> <error-type> <can-id>\n"
        try:
            sub_id = int(parts[0])
            error_type = int(parts[1], 0)
            can_id = int(parts[2].strip(), 0)
        except ValueError:
            return f"error: invalid args {args!r}\n"
        sub = self._ci_find_sub(sub_id)
        if sub is None:
            return f"error: subsystem {sub_id} not configured\n"
        self._publish_error_status(
            bus_address=self._args.bus_address,
            subsystem_id=sub.subsystem_id,
            error_type=error_type,
            section_id=0,
            interface_id=0,
            can_id=can_id,
        )
        logger.info("CI: emit ErrorStatus type=%d can_id=0x%03X for sub%d", error_type, can_id, sub_id)
        return "ok\n"

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
            elif cmd.startswith("set-state "):
                writer.write(self._ci_set_state(cmd[len("set-state "):]).encode())
            elif cmd.startswith("inject-fault "):
                writer.write(self._ci_inject_fault(cmd[len("inject-fault "):]).encode())
            elif cmd.startswith("emit-error "):
                writer.write(self._ci_emit_error(cmd[len("emit-error "):]).encode())
            elif cmd == "query-detailed":
                import json as _json
                import time as _time
                now = _time.monotonic()
                detail = {
                    str(s.subsystem_id): {
                        "state": s.state.name,
                        "state_int": int(s.state),
                        "fault_count": s.fault_count,
                        "fault_cause": s.fault_cause,
                        "last_rx_age_ms": round((now - s.last_rx_time) * 1000, 1),
                        "counter": s.counter,
                        "estop_latched": s.estop_latched,
                    }
                    for s in self._subsystems
                }
                writer.write((_json.dumps(detail) + "\n").encode())
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
            tg.create_task(self._report_data_loop())


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulated MCM CAN node (sygnal protocol)"
    )
    parser.add_argument("--can-interface", default="vcan0")
    # 200ms matches real MCM firmware (sygnal_docs v2.1.1 fault 0x30 "configured timeout").
    # Asyncio jitter is handled by tightened recv (20ms) and watchdog-check (10ms) intervals,
    # leaving ~170ms of slack. Raise to 2000ms for local dev on slow machines if needed.
    parser.add_argument("--watchdog-timeout-ms", type=int, default=200)
    parser.add_argument("--heartbeat-rate-hz", type=float, default=15.0)
    parser.add_argument("--report-rate-hz", type=float, default=10.0,
                        help="Rate (Hz) for synthetic ReportData signal broadcasts")
    parser.add_argument(
        "--devices",
        default=None,
        help=(
            "Comma-separated list of <address>:<type> device specs, e.g. '1:mcm,2:cb,3:io'. "
            "When set, takes precedence over --bus-address and --subsystem-ids."
        ),
    )
    # Legacy single-device flags — kept for backwards compatibility.
    # Ignored when --devices is provided.
    parser.add_argument("--bus-address", type=int, default=1)
    parser.add_argument(
        "--subsystem-ids",
        type=lambda s: [int(x) for x in s.split(",")],
        default=[0, 1],
        help="Comma-separated subsystem IDs to simulate (default: 0,1). Ignored if --devices is set.",
    )
    parser.add_argument(
        "--socket-path",
        default=None,
        help="Unix socket path for estop/query commands (default: /tmp/simulated-mcm-{bus_address}.sock)",
    )
    args = parser.parse_args()

    # Parse --devices string into list[DeviceSpec] if provided
    if args.devices is not None:
        args.devices = parse_devices_spec(args.devices)
    else:
        args.devices = None

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

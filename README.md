# mcm-simulator

Standalone Python package that emulates a sygnal MCM (Machine Control Module) on a SocketCAN interface. Install it anywhere, point it at a virtual or physical CAN bus, and get a fully functional MCM to test against — no external dependencies beyond `python-can` and `cantools`.

## What it does

The simulator speaks the sygnal CAN protocol on a virtual or physical CAN bus, responding to control frames exactly as real MCM hardware would:

- Publishes `Heartbeat` (0x170) at a configurable rate
- Responds to `ControlEnable` (0x60) with state transitions
- Responds to `ControlCommand` (0x160) and `RelayCommand` (0xB0)
- Enforces a watchdog timer — transitions to `FAIL_HARD` on timeout
- Implements fault-clear challenge-response (`HeartbeatClearSeed` / `HeartbeatClearKey`)
- Accepts e-stop / recovery via Unix socket
- Supports dual-subsystem mode (two independent state machines on one CAN bus)

## State Machine

```
FAIL_HARD(254)  <-- boot state
      |
      | HeartbeatClearKey (seed XOR 0xA5A5A5A5)
      v
HUMAN_CONTROL(0)  <-- ControlEnable Enable=0
      |
      | ControlEnable Enable=1
      v
MCM_CONTROL(1)
      |
      | watchdog timeout  OR  estop-press socket cmd
      v
FAIL_HARD(254)  [latched on estop, unlatchable via estop-release or fault clear]
```

MCM always boots in `FAIL_HARD`. The VCU must complete the fault-clear challenge before any control commands are accepted.

## Prerequisites

```sh
# vcan0 must exist before starting the simulator
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Install

```sh
pip install git+https://github.com/SygnalAuto/mcm-simulator.git

# Or from a local clone
pip install ./mcm-simulator
```

## Run

```sh
# Start with defaults (vcan0, 200ms watchdog, 15Hz heartbeat, subsystems 0,1)
mcm-simulator

# Raise watchdog for asyncio jitter on slow machines (real MCM uses 200ms)
mcm-simulator --watchdog-timeout-ms 2000

# Single subsystem
mcm-simulator --subsystem-ids 0

# Full options
mcm-simulator --help
```

### CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--can-interface` | `vcan0` | SocketCAN interface name |
| `--watchdog-timeout-ms` | `200` | Watchdog timeout in ms — matches real MCM (sygnal_docs v2.1.1). Raise to 2000ms on slow machines if asyncio jitter causes false FAIL_HARD trips. |
| `--heartbeat-rate-hz` | `15.0` | Heartbeat publish rate |
| `--bus-address` | `1` | MCM bus address in CAN frames |
| `--subsystem-ids` | `0,1` | Comma-separated subsystem IDs to simulate |
| `--socket-path` | `/tmp/simulated-mcm.sock` | Unix socket path for control commands |

### Inject commands via Unix socket

```sh
# While simulator is running:
echo "estop-press"    | nc -U /tmp/simulated-mcm.sock   # latch FAIL_HARD
echo "estop-release"  | nc -U /tmp/simulated-mcm.sock   # clear estop latch
echo "fail-sub0"      | nc -U /tmp/simulated-mcm.sock   # partial failure: sub0 only
echo "fail-sub1"      | nc -U /tmp/simulated-mcm.sock   # partial failure: sub1 only
echo "recover-sub0"   | nc -U /tmp/simulated-mcm.sock   # recover sub0
echo "recover-sub1"   | nc -U /tmp/simulated-mcm.sock   # recover sub1
echo "query"          | nc -U /tmp/simulated-mcm.sock   # get JSON state of all subsystems
```

### CI Hooks — deterministic state and fault injection

The following commands let CI tests drive the simulator into specific protocol states and verify how interfacing applications react.

```sh
SOCK=/tmp/simulated-mcm-1.sock

# Force subsystem 0 into any documented state
echo "set-state 0 FAIL_HARD"       | nc -U $SOCK   # → FAIL_HARD (254)
echo "set-state 0 HUMAN_CONTROL"   | nc -U $SOCK   # → HUMAN_CONTROL (0)
echo "set-state 0 MCM_CONTROL"     | nc -U $SOCK   # → MCM_CONTROL (1)
echo "set-state 0 FAIL_OP_1"       | nc -U $SOCK   # → FAIL_OPERATIONAL_1 (241)
echo "set-state 0 FAIL_OP_2"       | nc -U $SOCK   # → FAIL_OPERATIONAL_2 (242)
echo "set-state 0 HUMAN_OVERRIDE"  | nc -U $SOCK   # → HUMAN_OVERRIDE (253)

# Inject a specific fault cause (increments counter, emits FaultIncrement frame)
echo "inject-fault 0 0x30"         | nc -U $SOCK   # Heartbeat Timeout fault on sub0
echo "inject-fault 1 0x01"         | nc -U $SOCK   # Emergency Stop fault on sub1

# Emit a one-shot ErrorStatus frame (does not change state)
echo "emit-error 0 1 0x60"         | nc -U $SOCK   # CRC Error on ControlEnable
echo "emit-error 0 6 0x160"        | nc -U $SOCK   # State Error on ControlCommand

# Detailed JSON query (all subsystems)
echo "query-detailed"              | nc -U $SOCK
# Returns: {"0": {"state": "HUMAN_CONTROL", "state_int": 0, "fault_count": 0,
#                 "fault_cause": 0, "last_rx_age_ms": 12.3, "counter": 42,
#                 "estop_latched": false}, ...}
```

All commands respond with `ok` on success or `error: <reason>` on failure.

## Programmatic use (CI)

```python
import asyncio
import argparse
from mcm_simulator import McmSimulator, SystemState

def make_args(**kwargs):
    defaults = dict(
        can_interface="vcan0",
        watchdog_timeout_ms=200,
        heartbeat_rate_hz=15.0,
        bus_address=1,
        subsystem_ids=[0, 1],
        socket_path="/tmp/simulated-mcm.sock",
    )
    defaults.update(kwargs)
    return argparse.Namespace(**defaults)

async def run_test():
    sim = McmSimulator(make_args())
    task = asyncio.create_task(sim.run())
    # ... run your VCU tests ...
    task.cancel()

asyncio.run(run_test())
```

## CRC8 Warning

The MCM silently drops all frames with incorrect CRC — no error is logged, no diagnostic is emitted. If the simulator appears unresponsive after startup, run the CRC test vectors first:

```sh
pytest tests/test_crc8.py -v
```

## Tests

```sh
pip install "mcm-simulator[dev]"
pytest tests/ -q
```

Tests mock `python-can` — no real vcan0 required.

## DBC files

The bundled DBC files in `mcm_simulator/dbc/` are copied from the sygnal project:
- Source: https://github.com/polymathrobotics/sygnal
- Commit: `ec0336b044ca04183e479a214d4878a5b112b0a8`
- License: Apache 2.0

Files: `Heartbeat.dbc`, `Control.dbc`, `Relay.dbc` (the 3 of 11 sygnal MCM DBC files used by this simulator).

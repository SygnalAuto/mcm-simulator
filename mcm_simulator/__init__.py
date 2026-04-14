"""mcm-simulator — standalone sygnal MCM CAN simulator.

Emulates real MCM hardware on a SocketCAN interface (vcan0 or physical CAN).
Speaks the sygnal CAN protocol: heartbeat, control enable/command/relay, fault
clear challenge-response, watchdog, and e-stop.

Entry points:
  CLI:         mcm-simulator  (see simulator.py:main)
  Programmatic: McmSimulator, SubsystemState, SystemState
"""

from mcm_simulator.simulator import McmSimulator, SubsystemState, SystemState

__all__ = ["McmSimulator", "SubsystemState", "SystemState"]

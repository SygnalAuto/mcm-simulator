"""In-memory configuration store for the mcm-simulator.

Maps (bus_address, subsystem_id, interface_id, section_id, param_id) → float value.
"""

from __future__ import annotations


ConfigKey = tuple[int, int, int, int, int]


class ConfigStore:
    """Thread-safe-by-simplicity (single-threaded asyncio) config value store."""

    def __init__(self) -> None:
        self._store: dict[ConfigKey, float] = {}

    def get(
        self,
        bus_address: int,
        subsystem_id: int,
        interface_id: int,
        section_id: int,
        param_id: int,
    ) -> float:
        key: ConfigKey = (bus_address, subsystem_id, interface_id, section_id, param_id)
        return self._store.get(key, 0.0)

    def set(
        self,
        bus_address: int,
        subsystem_id: int,
        interface_id: int,
        section_id: int,
        param_id: int,
        value: float,
    ) -> None:
        key: ConfigKey = (bus_address, subsystem_id, interface_id, section_id, param_id)
        self._store[key] = value

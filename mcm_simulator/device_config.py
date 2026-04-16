"""Device identity configuration for the mcm-simulator.

Defines DeviceSpec (bus address, product type, firmware versions, serial number)
and a parser for the --devices CLI flag format: "1:mcm,2:cb,3:io".
"""

from __future__ import annotations

from dataclasses import dataclass, field


#: Maps device type name → Sygnal ProductID
PRODUCT_IDS: dict[str, int] = {
    "mcm": 1,
    "cb": 5,
    "io": 6,
}


@dataclass
class DeviceSpec:
    """Identity and configuration for a single simulated Sygnal device."""

    bus_address: int
    product_type: str  # "mcm", "cb", or "io"
    product_id: int
    subsystem_ids: list[int] = field(default_factory=lambda: [0, 1])
    serial_number: int = 0
    fw_major: int = 1
    fw_minor: int = 0
    fw_patch: int = 0
    bl_major: int = 1
    bl_minor: int = 0
    bl_patch: int = 0

    def __post_init__(self) -> None:
        if self.serial_number == 0:
            # Auto-generate a non-zero serial derived from bus address
            self.serial_number = 1000 + self.bus_address


def parse_devices_spec(spec_str: str) -> list[DeviceSpec]:
    """Parse a devices spec string into a list of DeviceSpec objects.

    Format: "<addr>:<type>[,<addr>:<type>...]"
    Example: "1:mcm,2:cb,3:io"

    Raises ValueError for invalid formats, duplicate addresses, or unknown types.
    """
    if not spec_str or not spec_str.strip():
        raise ValueError("Empty devices spec: expected at least one '<addr>:<type>'")

    devices: list[DeviceSpec] = []
    seen_addresses: set[int] = set()

    for part in spec_str.split(","):
        part = part.strip()
        if not part:
            continue

        if ":" not in part:
            raise ValueError(
                f"Invalid device spec '{part}': expected '<address>:<type>' (e.g., '1:mcm')"
            )

        addr_str, type_str = part.split(":", 1)
        addr_str = addr_str.strip()
        type_str = type_str.strip().lower()

        try:
            addr = int(addr_str)
        except ValueError:
            raise ValueError(
                f"Invalid bus address '{addr_str}': must be an integer"
            ) from None

        if type_str not in PRODUCT_IDS:
            valid = sorted(PRODUCT_IDS)
            raise ValueError(f"Unknown device type '{type_str}': must be one of {valid}")

        if addr in seen_addresses:
            raise ValueError(
                f"Duplicate bus address {addr}: each device must have a unique address"
            )

        seen_addresses.add(addr)
        devices.append(
            DeviceSpec(
                bus_address=addr,
                product_type=type_str,
                product_id=PRODUCT_IDS[type_str],
            )
        )

    if not devices:
        raise ValueError("No valid devices in spec string")

    return devices

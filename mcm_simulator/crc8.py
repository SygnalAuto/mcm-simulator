"""CRC-8/CCITT implementation matching sygnal generate_crc8.

Standalone module shared by simulator and tests so CRC8 correctness
is verifiable independently of CAN bus availability.

Test vectors from sygnal spec must pass before CAN simulator integration:
all-zeros->0x00, HUMAN_CONTROL counter=1->0x29, MCM_CONTROL counter=5->0x52, FAIL_HARD counter=10->0x86.

WARNING: sygnal silently drops all frames with incorrect CRC — no error, no log.
A CRC mismatch causes the simulated MCM to become invisible to sygnal with zero feedback.
"""

from __future__ import annotations


def generate_crc8(data: bytes | bytearray) -> int:
    """Compute CRC-8 over first 7 bytes of an 8-byte CAN frame.

    Polynomial 0x07, initial value 0x00, MSB-first, no reflection, no final XOR.
    Matches sygnal generate_crc8 in crc8.cpp.
    """
    crc = 0x00
    for i in range(7):
        crc ^= data[i]
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def apply_crc8(data: bytearray) -> bytearray:
    """Set data[7] to the CRC-8 of data[0:7] and return the frame."""
    data[7] = generate_crc8(data)
    return data

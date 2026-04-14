"""CRC-8 test vectors from sygnal spec.

Mandatory: all vectors must pass before CAN simulator integration.
sygnal silently drops frames with incorrect CRC — a mismatch makes the
simulator invisible with zero feedback.
"""

from mcm_simulator.crc8 import apply_crc8, generate_crc8


class TestGenerateCrc8:
    def test_all_zeros(self):
        data = bytes(7)
        assert generate_crc8(data) == 0x00

    def test_human_control_counter_1(self):
        # Mandatory vector: HUMAN_CONTROL(0) BusAddress=0 Counter=1
        data = bytes([0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        assert generate_crc8(data) == 0x29

    def test_mcm_control_counter_5(self):
        # Mandatory vector: MCM_CONTROL(1) BusAddress=1 Counter=5
        data = bytes([0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00])
        assert generate_crc8(data) == 0x52

    def test_fail_hard_counter_10(self):
        # Mandatory vector: FAIL_HARD(254=0xFE) BusAddress=0 Counter=10
        data = bytes([0xFE, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00])
        assert generate_crc8(data) == 0x86

    def test_accepts_bytearray(self):
        data = bytearray(7)
        assert generate_crc8(data) == 0x00


class TestApplyCrc8:
    def test_sets_byte_7(self):
        frame = bytearray(8)
        result = apply_crc8(frame)
        assert result[7] == 0x00

    def test_sets_byte_7_human_control(self):
        frame = bytearray([0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        result = apply_crc8(frame)
        assert result[7] == 0x29

    def test_returns_same_object(self):
        frame = bytearray(8)
        result = apply_crc8(frame)
        assert result is frame

    def test_roundtrip_nonzero(self):
        frame = bytearray([0xFE, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        apply_crc8(frame)
        assert generate_crc8(frame[:7]) == frame[7]

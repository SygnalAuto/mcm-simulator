"""Regression test: 200ms watchdog default does not trip under asyncio scheduler jitter.

Simulates a VCU sending frames at 50 Hz (20ms gap) while the watchdog
runs at its real 10ms check interval. Asserts no spurious FAIL_HARD
transitions over a 1 second run.
"""

from __future__ import annotations

import asyncio
import time

import pytest

from mcm_simulator.simulator import SubsystemState, SystemState


@pytest.mark.asyncio
async def test_no_spurious_fail_hard_at_200ms_watchdog() -> None:
    """200ms watchdog with 20ms feed gap: no spurious trips over 1 second."""
    watchdog_timeout_s = 0.200
    check_interval = 0.010  # 10ms — matches Task 2 tightened interval
    feed_interval = 0.020   # 20ms — 50Hz VCU heartbeat

    sub = SubsystemState(
        subsystem_id=0, bus_address=1, watchdog_timeout_s=watchdog_timeout_s
    )
    sub.state = SystemState.HUMAN_CONTROL  # Not in FAIL_HARD — watchdog is active

    end = time.monotonic() + 1.0
    last_feed = time.monotonic()
    trips = 0

    while time.monotonic() < end:
        now = time.monotonic()

        # Feed watchdog at 50Hz
        if now - last_feed >= feed_interval:
            sub.feed_watchdog()
            last_feed = now

        # Run watchdog check
        if sub.check_watchdog():
            trips += 1

        await asyncio.sleep(check_interval)

    assert trips == 0, f"Spurious FAIL_HARD trips: {trips} (should be 0)"
    assert sub.state == SystemState.HUMAN_CONTROL


def test_no_spurious_fail_hard_synchronous() -> None:
    """Synchronous variant using asyncio.run — works without pytest-asyncio."""
    asyncio.run(test_no_spurious_fail_hard_at_200ms_watchdog())

# Bus Address Filtering Implementation Plan

Created: 2026-04-14
Status: VERIFIED
Approved: Yes
Iterations: 0
Worktree: No
Type: Feature

## Summary

**Goal:** Make the MCM simulator explicitly filter incoming CAN messages by bus address, so multiple simulator instances can coexist on the same bus without processing each other's messages. Also make the Unix socket path unique per instance to prevent bind conflicts.

**Architecture:** The `--bus-address` CLI arg already exists (default=1). The change restructures `_handle_can_message` to decode first, then filter by `BusAddress` field before feeding the watchdog or processing. The default socket path becomes dynamic based on bus address.

**Tech Stack:** Python, python-can, cantools, argparse

## Scope

### In Scope

- Filter incoming CAN messages by `BusAddress` field in `_handle_can_message`
- Restructure decode/watchdog order so foreign messages don't feed the watchdog
- Make default socket path include bus address (`/tmp/simulated-mcm-{N}.sock`)
- Tests for bus address filtering behavior
- Update existing test fixtures if affected

### Out of Scope

- Bus address range validation (any int is accepted, matches CAN protocol flexibility)
- CAN interface sharing concerns (SocketCAN handles this natively)
- Changes to outgoing message encoding (already correct)

## Approach

**Chosen:** Decode-then-filter — move message decode before watchdog feed, add bus address check between decode and processing.

**Why:** Matches real hardware behavior where only messages addressed to this node keep it alive. Prevents foreign traffic from resetting the watchdog timer, which would mask communication failures in multi-MCM setups.

**Alternatives considered:**
- *Filter after existing decode position* — simpler change but foreign messages would still feed the watchdog, giving incorrect behavior when multiple MCMs share a bus. Rejected because the watchdog should only be fed by messages actually intended for this node.

## Context for Implementer

> Write for an implementer who has never seen the codebase.

- **Patterns to follow:** The simulator is a single-file async CAN node (`mcm_simulator/simulator.py`). All CAN message handling flows through `_handle_can_message` (line ~395). Outgoing messages already encode `self._args.bus_address` — see `_send_control_enable_response`, `_publish_subsystem_heartbeat`, etc.
- **Conventions:** Args are accessed via `self._args` (argparse.Namespace). DBC files define message schemas with `BusAddress` as a signal in every message type. Tests use `_make_args()` helper to construct args and `_make_can_frame()` to build CAN frames.
- **Key files:**
  - `mcm_simulator/simulator.py` — main simulator: `_parse_args()` (line 464), `_handle_can_message()` (line ~395), `McmSimulator.__init__()` (line 158)
  - `tests/test_simulator.py` — all tests, `_make_args()` (line 25), `_make_can_frame()` (line ~45)
  - `dbc/*.dbc` — CAN message definitions (Heartbeat.dbc, Control.dbc, Relay.dbc)
- **Gotchas:**
  - `decoded.get("BusAddress")` returns a float from cantools — cast to `int()` before comparing
  - HeartbeatClearKey intentionally does NOT feed the watchdog (existing behavior, preserve this)
  - The `_make_args` test helper hardcodes `socket_path="/tmp/test-simulated-mcm.sock"` — this is intentional for tests and should not change

## Assumptions

- All incoming CAN message types in `_USER_APP_IDS` have a `BusAddress` signal defined in their DBC schema — supported by test code showing `BusAddress` in all `_make_can_frame` calls — Tasks 1, 2 depend on this
- The `bus_address` field in decoded messages is numeric (int/float) — supported by cantools decode behavior — Task 1 depends on this
- Existing tests all use `BusAddress: 1` in frames and `bus_address=1` in args — supported by reading test code — Task 1 depends on this (existing tests should pass without modification)

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| A DBC message type lacks BusAddress signal | Low | High — KeyError crash | Use `decoded.get("BusAddress", 0)` with fallback; if 0 != configured address, frame is dropped (safe default) |
| cantools returns BusAddress as float | Medium | Medium — comparison fails | Always cast with `int()` before comparing |
| Existing tests break from decode reordering | Low | Low — test-only | All existing tests use matching bus address; reordering decode vs watchdog feed doesn't affect them |

## Goal Verification

### Truths

1. Running `--bus-address 2` causes the simulator to ignore CAN frames with `BusAddress=1`
2. Running `--bus-address 2` causes the simulator to process CAN frames with `BusAddress=2`
3. Foreign-address frames do NOT feed the watchdog timer
4. Default socket path is `/tmp/simulated-mcm-{N}.sock` where N is the bus address
5. Explicitly passing `--socket-path` overrides the dynamic default
6. All existing tests pass without modification

### Artifacts

- `mcm_simulator/simulator.py` — bus address filter in `_handle_can_message`, dynamic socket path in `_parse_args`
- `tests/test_simulator.py` — new tests for bus address filtering and socket path

## Progress Tracking

- [x] Task 1: Bus address filtering in _handle_can_message
- [x] Task 2: Dynamic socket path default
      **Total Tasks:** 2 | **Completed:** 2 | **Remaining:** 0

## Implementation Tasks

### Task 1: Bus Address Filtering in _handle_can_message

**Objective:** Restructure `_handle_can_message` to decode messages before the watchdog feed, then filter by `BusAddress` field. Only messages matching the configured bus address are processed.

**Dependencies:** None

**Files:**

- Modify: `mcm_simulator/simulator.py` — restructure `_handle_can_message`
- Modify: `tests/test_simulator.py` — add filtering tests

**Key Decisions / Notes:**

- Move the `self._db.decode_message()` call BEFORE the watchdog feed block
- Add `if int(decoded.get("BusAddress", 0)) != self._args.bus_address: return` between decode and watchdog feed
- The HeartbeatClearKey watchdog exemption stays the same — it's about message type, not address
- Existing tests all use `BusAddress: 1` matching `bus_address=1` default, so they pass unchanged
- Use `decoded.get("BusAddress", 0)` not `decoded["BusAddress"]` for safety

**Definition of Done:**

- [x] `_handle_can_message` decodes before feeding watchdog
- [x] Messages with non-matching BusAddress are silently dropped
- [x] Foreign-address messages do not feed the watchdog
- [x] All existing tests pass unchanged
- [x] New test: frame with wrong BusAddress is ignored (state unchanged, watchdog not fed)
- [x] New test: frame with correct BusAddress is processed normally

**Verify:**

- `uv run pytest tests/test_simulator.py -q`

### Task 2: Dynamic Socket Path Default

**Objective:** Change the default `--socket-path` to include the bus address, preventing bind conflicts when running multiple simulator instances.

**Dependencies:** None (can be done in parallel with Task 1)

**Files:**

- Modify: `mcm_simulator/simulator.py` — change `_parse_args` socket-path default
- Modify: `tests/test_simulator.py` — add test for dynamic socket path

**Key Decisions / Notes:**

- Change `--socket-path` default to `None` (sentinel)
- After `parser.parse_args()`, compute default: `f"/tmp/simulated-mcm-{args.bus_address}.sock"` when None
- Explicit `--socket-path /custom/path` still overrides
- Test `_make_args` already hardcodes `socket_path="/tmp/test-simulated-mcm.sock"` — no change needed there

**Definition of Done:**

- [x] Default socket path is `/tmp/simulated-mcm-{bus_address}.sock` (e.g., `/tmp/simulated-mcm-1.sock` for `--bus-address 1`)
- [x] Explicit `--socket-path` overrides the dynamic default
- [x] New test: default socket path includes bus address
- [x] New test: explicit socket path is preserved
- [x] All existing tests pass

**Verify:**

- `uv run pytest tests/test_simulator.py -q`

## Deferred Ideas

- Bus address range validation (e.g., 1-255) — add if real protocol spec constrains this
- Log bus address at startup for operator visibility (could be a follow-up)

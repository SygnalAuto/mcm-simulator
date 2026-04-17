# Align MCM Simulator with sygnal_docs Implementation Plan

Created: 2026-04-17
Author: joshhartung@gmail.com
Status: VERIFIED
Approved: Yes
Iterations: 0
Worktree: No
Type: Feature

## Summary

**Goal:** Audit `mcm-simulator` against `~/sygnal_docs` (v2.1.1 of MCM firmware) and fix clear bugs where the simulator diverges from documented protocol behavior. Missing features are catalogued, not implemented.

**Architecture:** The simulator is a single long-running asyncio daemon (`mcm_simulator/simulator.py`) that speaks the Sygnal CAN protocol against a SocketCAN interface. Per-subsystem state machines live in `SubsystemState`; device-level concerns (bus addressing, DBC decode, socket server) live in `McmSimulator`. All CAN frame layouts are defined by DBC files in `mcm_simulator/dbc/`.

**Tech Stack:** Python 3.12, `python-can`, `cantools`, asyncio, pytest, uv.

**Branch:** `feat/align-simulator-with-sygnal-docs` (created from `origin/main`).

## Scope

### In Scope
- **CAN API + frame layouts** — validate IDs, byte layouts, and signal packing against `mcm-can-api-guide.md` + DBC files
- **State machine + faults + human-user-override** — validate state values, fault causes, clear-challenge behavior
- **Signal reporting + diagnostics + virtual interfaces** — validate ReportData signal IDs, FaultState/FaultList/FaultRootCause/ErrorStatus publishing
- **Seven clear-bug fixes** (Tasks 1–7) approved in Step 7 batch
- **One CI test surface extension** (Task 8 — added per user feedback at approval gate) — extends Unix socket protocol with deterministic state / fault / error injection so CI tests for interfacing apps can reach states the sim doesn't enter organically
- **Audit catalogue embedded in this plan file** documenting every doc-↔-sim divergence found

### Out of Scope
- **Signal generation, closed-loop control, relay control FEATURES** — user deferred these doc areas. NOTE: the shared dispatch path in `_handle_can_message` IS touched (Task 5 applies the same fault gate + watchdog-feed suppression to RelayCommand that it applies to ControlEnable/ControlCommand) because gating one without the other leaves a silent correctness hole. This is a protocol-consistency fix on a shared code path, not an audit of relay-control.md.
- **Full Human Override state + 0x40/0x41 CAN messages** — missing feature, catalog only
- **FAIL_OPERATIONAL_1/2 state *transitions*** (we add the enum values but no code ever enters them)
- **OSCC API** (0x70–0x93 brake/steering/throttle legacy protocol) — missing feature, catalog only
- **FaultState receive-side propagation** (docs say incoming FaultState frames increment local counter) — catalog only
- **Installation, hardware, safety-documentation, commissioning password, firmware updates** — not simulator concerns
- **DBC edits** — docs say "refer to DBC for most up to date information," so DBC wins on conflicts

## Approach

**Chosen:** Surgical fixes on the existing `simulator.py` module, preserving current architecture (single `McmSimulator` class, per-subsystem `SubsystemState`). Audit findings live in a `## Docs Compliance Audit` section of this plan rather than a separate artifact.

**Why:** The sim's structure already maps cleanly to the docs (one subsystem state machine per MCM subsystem, one frame handler per CAN ID). The fixes are small, localized, and each has an obvious DoD. Embedding the audit in-plan keeps context with the work for reviewers and avoids a stray doc artifact users must maintain.

**Alternatives considered:**
- *Rewrite the state machine to model all six documented states with real transitions* — rejected, would expand scope beyond "catalog gaps only" and require modeling HUO detection logic that has no hardware analog in the simulator.
- *Separate `docs/DOCS_COMPLIANCE.md` artifact* — rejected per user choice; embed in plan.
- *Fix the DBC files to match docs tables* — rejected; docs explicitly defer to DBC as canonical.

## Context for Implementer

- **Entry point:** `mcm_simulator/simulator.py:984 main()` → `McmSimulator.run()` at line 923 runs six asyncio tasks concurrently.
- **CAN ID constants:** `simulator.py:56–80` — preserve these names when used elsewhere in tests.
- **Fault cause IDs:** `SubsystemState._FAULT_CAUSE_*` at `simulator.py:122–124`. Docs define many more; sim only maps three.
- **Canonical-source framing** (refined after Codex review): `sygnal_docs` firmware v2.1.1 is the source of truth for WHICH message families exist and WHAT they mean. The bundled DBCs in `mcm_simulator/dbc/` are an intentional subset of the upstream Sygnal DBC — they do NOT contain every documented message (Override 0x40/0x41 and OSCC API are absent). So: (a) for messages present in BOTH docs and DBC, DBC wins on byte layout since the VCU decodes with the same DBC file and so matches the sim; (b) for messages in docs but missing from DBC, treat as 📋 catalogued gaps regardless of how a future DBC edit would resolve them; (c) every DBC-vs-docs divergence on a shared message is recorded in § Docs Compliance Audit. Examples already identified:
  - `ErrorStatus` (0x30) — DBC has `Config_SectionID`/`Config_InterfaceID` in byte 1 that docs omit. DBC wins for encoding (sim + VCU agree via DBC); audit flags the docs-table discrepancy.
  - `Config_SectionID` VAL_TABLE — DBC has `31 "Report"`, docs claim `0x10`. Follow DBC (31); audit flags.
  - `Override` (0x40/0x41) — in docs, not in DBC — 📋 catalogued, not implemented.
- **CRC:** all 8-byte frames use `crc8.py` (poly 0x07) over bytes 0–6. `apply_crc8(frame)` mutates `frame[7]`.
- **Testing:** `uv run pytest -q` is the canonical test runner (per `~/.claude/rules/standards-python.md`). Tests are in `tests/`; mock the CAN bus (`sim._bus = MagicMock()`).
- **Gotchas:**
  - `_handle_can_message` filters by `self._args.bus_address` (primary device's address only). For multi-device mode, per-device routing only exists inside `_handle_config_command`. Do NOT widen this filter as part of this plan — noted in audit.
  - `_fault_clear_seed_loop` and `_handle_heartbeat_clear_key` only iterate `self._subsystems` (primary device) — same multi-device caveat.
  - Heartbeat is published for **all** device nodes, but watchdog and fault-clear only operate on primary. Noted in audit.
  - `_publish_error_status` exists at `simulator.py:487` but is never called. Task 4 wires it up.
  - `Worktree: No` in the header — we're on a fresh branch `feat/align-simulator-with-sygnal-docs` but not in an isolated worktree.

## Assumptions

- The `~/sygnal_docs` repo is the current canonical reference for MCM firmware v2.1.1 behavior — supported by `overview.md:3` stating "version 2.1.1". All tasks depend on this.
- The DBC files in `mcm_simulator/dbc/` are the authoritative signal layouts — supported by `mcm-can-api-guide.md:3` ("refer to the CAN DBC files provided in Downloads"). Tasks 1, 3, 4, 5, 6 assume this.
- Existing test suite passes on `main` before these changes — will verify at task start. All tasks depend on a clean baseline.
- Python 3.12+ and `uv` are available per `pyproject.toml`. All tasks depend on this.
- No downstream VCU code currently relies on the wheel-speed report signals (0x70–0x74) the sim broadcasts — supported by these IDs not appearing in any doc page. Task 7 depends on this.
- Tests use `MagicMock()` for the CAN bus, so changes to frame emission timing do not break test assertions unless the test explicitly asserts emission count. Tasks 3, 5, 6 rely on this.

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Changing watchdog default to 200 ms flakes tests on slow CI — raised by Codex as a high-likelihood concern given current 100 ms recv + 50 ms check tick | Medium | Medium | Task 2 addresses this by ALSO tightening async recv timeout (100 → 20 ms) and watchdog check interval (50 → 10 ms), leaving 170 ms of slack under the 200 ms default. Task 2 adds `test_watchdog_jitter.py` as a regression guard. Tests that explicitly need 2000 ms must pass it via fixture. |
| Reject-control-in-fault (Task 5) breaks existing integration tests that send ControlCommand in FAIL_HARD and expect ControlCommandResponse | High | Medium | Audit `test_simulator.py` and `test_multi_device.py` for those patterns; update expectations to assert `ErrorStatus` frame. |
| FaultList/FaultRootCause publishers (Task 6) fire too often and flood logs in tests | Low | Low | Publishers gated on state ∈ {FAIL_HARD, FAIL_OPERATIONAL_*, HUMAN_OVERRIDE}; since only FAIL_HARD is ever reached, practical rate stays ≤ 2 Hz. |
| Widening `SystemState` enum (Task 1) breaks `int(sub.state)` comparisons in tests | Low | Low | Enum is `IntEnum`; new values are additive. Verify with `test_diagnostics_frames.py` FaultState assertions. |
| `_publish_error_status` byte layout conflict (DBC vs docs) confuses VCU consumers | Medium | Low | DBC is canonical; VCU decodes via the same DBC. Note the conflict in audit. Add a unit test that round-trips ErrorStatus frames through the DBC decoder. |
| Replacing report signal IDs (Task 7) breaks any consumer relying on 0x70–0x74 wheel speeds | Low | Medium | Docs don't list those IDs, so no compliant VCU uses them. Keep existing `_synthetic_value` shape; only swap IDs + labels. README callout. |
| 2 pre-existing test failures on `main` (`test_heartbeat_sent_for_all_devices`, `test_multiple_subsystems_per_device` in `test_multi_device.py`) | Certain | Low | Baseline captured before implementation. DoD phrasing "full suite passes" means "no NEW failures introduced by these tasks." The 2 pre-existing failures are multi-device subsystem fan-out issues unrelated to docs compliance and are out of scope for this plan. Each task's verify step confirms delta=0 vs. baseline. |

## Goal Verification

### Truths

1. `uv run pytest -q` shows no NEW failures after all tasks complete — only the 2 pre-existing `test_multi_device.py` baseline failures remain (captured before implementation).
2. `mcm_simulator/simulator.py SystemState` defines exactly six values: `HUMAN_CONTROL=0`, `MCM_CONTROL=1`, `FAIL_OPERATIONAL_1=241`, `FAIL_OPERATIONAL_2=242`, `HUMAN_OVERRIDE=253`, `FAIL_HARD=254`, matching `Heartbeat.dbc` VAL_TABLE_.
3. `_parse_args`'s `--watchdog-timeout-ms` default is `200`; `_can_receive_loop` recv timeout is 20 ms; `_watchdog_loop` check interval is 10 ms; `test_watchdog_jitter.py` passes reliably (no false FAIL_HARD trips under 1 s run with 20 ms heartbeat gap).
4. `FaultState` (0x20) frames are only emitted when the subsystem's state is in {FAIL_HARD, FAIL_OPERATIONAL_1, FAIL_OPERATIONAL_2, HUMAN_OVERRIDE} — verified by a unit test.
5. A CRC-mismatched CAN frame triggers an `ErrorStatus` (0x30) emission with `Error_Type = 1` (CRC Error), verified by unit test.
6. `ControlEnable` (0x60), `ControlCommand` (0x160), and `RelayCommand` (0xB0) received while any subsystem is in `FAIL_HARD` each produce `ErrorStatus` frames (Error_Type 6 "State Error") instead of normal responses, AND the watchdog is NOT fed for those rejected frames — verified by unit tests.
7. While any subsystem is in an active fault state, `FaultList` (0x220) and `FaultRootCause` (0x221) frames appear on the bus at the seed rate — verified by unit tests asserting emission + correct encoding.
8. `ReportData` (0x700) synthetic broadcasts use only Signal IDs listed in `mcm-can-api-guide.md` § Report (0x80, 0x81, 0x83, 0x50, 0x52, 0x40) — verified by unit test enumerating published IDs.
9. This plan file contains a complete `## Docs Compliance Audit` section listing every divergence between docs and simulator, with severity and disposition (fixed / catalogued / deferred).
10. The Unix socket server accepts new CI commands `set-state`, `inject-fault`, `emit-error`, `query-detailed` per Task 8; an external script can drive a subsystem into any of the 6 `SystemState` values, inject any of the 0x01–0x42 fault causes, emit any 0x01–0x0C ErrorStatus frame, and query per-subsystem internal state — all verified by `tests/test_ci_hooks.py`.

### Artifacts

- `mcm_simulator/simulator.py` — implementation changes for Tasks 1–7
- `tests/test_diagnostics_frames.py` — expanded assertions for Tasks 3, 5, 6
- `tests/test_simulator.py` — assertions for Tasks 2, 4, 5
- `tests/test_signal_reporting.py` *(new)* — Task 7 assertions; also documents signal ID expectations
- `README.md` — default watchdog callout + report signal ID list update
- This plan file — `## Docs Compliance Audit` section

## Docs Compliance Audit

**Legend:** ✅ fixed by this plan | ⚠ caveat / partial | 📋 catalogued gap, not implemented | ➖ out of scope

### 1. State Machine (`states.md`, `simulator.py:49`)

| Doc state | Value | Sim status |
|-----------|-------|------------|
| Human Control | 0x00 | ✅ implemented |
| MCM Control | 0x01 | ✅ implemented |
| Fail Operational 1 | 0xF1 (241) | ✅ enum value added (Task 1); 📋 no transition logic — sim never enters this state |
| Fail Operational 2 | 0xF2 (242) | ✅ enum value added (Task 1); 📋 no transition logic |
| Human Override | 0xFD (253) | ✅ enum value added (Task 1); 📋 no override detection — sim has no analog signal path |
| Fail Hard | 0xFE (254) | ✅ implemented |

### 2. Fault Sources (`faults-and-fault-behavior.md`, `simulator.py:122`)

| Fault ID | Name | Sim status |
|----------|------|------------|
| 0x01 | Emergency Stop | ✅ `_FAULT_CAUSE_ESTOP` — used |
| 0x02 | External Fault | ⚠ `_FAULT_CAUSE_EXTERNAL` constant defined but no trigger path — no "external fault" Unix-socket cmd exists |
| 0x03 | CAN Fail Op 1 State Message | 📋 no receive-side FaultState handling |
| 0x04 | CAN Fail Op 2 State Message | 📋 no receive-side FaultState handling |
| 0x05 | CAN Fail Hard State Message | 📋 no receive-side FaultState handling |
| 0x10 | Invalid Subsystem ID | 📋 no EEPROM/pin subsystem ID check |
| 0x11 | Invalid Configuration CRC | 📋 `ConfigStore` has no CRC |
| 0x1B | Physical Interface Transition | ➖ signal-generation scope (deferred) |
| 0x20 | Invalid Input Signal | ➖ signal-generation |
| 0x21 | Invalid Output Signal | ➖ signal-generation |
| 0x30 | Customer Application Heartbeat Timeout | ✅ `_FAULT_CAUSE_WATCHDOG` — used on watchdog |
| 0x31 | Customer Application Heartbeat Count | 📋 sim ignores Count8 field on ControlCommand |
| 0x32 | Control Timeout | ⚠ partially covered by 0x30 (watchdog resets on any rx) |
| 0x33 | Control Count | 📋 sim doesn't validate Count8 field |
| 0x34 | Control Out of Bounds | 📋 sim doesn't validate Value range |
| 0x35–0x37 | Closed Loop * | ➖ closed-loop-control scope (deferred) |
| 0x38 | Invalid Configuration Option | 📋 `ConfigStore` accepts any value |
| 0x40–0x42 | SW * | 📋 internal, not simulator-relevant |

### 3. CAN API — Frame-by-frame (`mcm-can-api-guide.md`)

| CAN ID | Name | Sim status |
|--------|------|------------|
| 0x20 FaultState (MCM→VCU) | ✅ published, but **currently emitted every heartbeat cycle** regardless of state. Fix in Task 3: only publish when state ∈ {fault states}. |
| 0x21 FaultIncrement (MCM→VCU) | ✅ published once per fault transition (correct) |
| 0x220 FaultList (MCM→VCU) | 📋 CAN ID constant defined at `simulator.py:68`, no publisher. **Task 6: implement at seed rate.** |
| 0x221 FaultRootCause (MCM→VCU) | 📋 CAN ID constant defined at `simulator.py:69`, no publisher. **Task 6: implement at seed rate.** |
| 0x30 ErrorStatus (MCM→VCU) | ⚠ `_publish_error_status` method exists but never called. **Task 4: wire up on CRC mismatch, unknown section/param.** ⚠ DBC byte layout diverges from docs table (DBC has SectionID+InterfaceID in byte 1); DBC is canonical per user decision. |
| 0x40 OverrideStatus (MCM→VCU) | 📋 not implemented — requires HUO detection |
| 0x41 OverrideAcknowledge (VCU→MCM) | 📋 not implemented |
| 0x60 ControlEnable (VCU→MCM) | ✅ decoded; ⚠ sim acts on it even in FAIL_HARD (**Task 5: reject with ErrorStatus**) |
| 0x61 ControlEnableResponse (MCM→VCU) | ⚠ sent from `_primary` only, should iterate subsystems (**Task 5** bundles this fix) |
| 0x70 HeartbeatClearSeed (MCM→VCU) | ✅ published every 500 ms when any sub is in FAIL_HARD; 📋 docs call this "seed rate" — configurability deferred |
| 0x7F HeartbeatClearKey (VCU→MCM) | ✅ validates XOR 0xA5A5A5A5 |
| 0x160 ControlCommand (VCU→MCM) | ✅ decoded; ⚠ sim responds even in FAIL_HARD; doesn't validate Count8 or Value bounds (**Task 5: reject in fault; Count/bounds catalogued**) |
| 0x161 ControlCommandResponse (MCM→VCU) | ⚠ from `_primary` only, Count8 echoed verbatim (**Task 5**) |
| 0x170 Heartbeat (MCM→VCU) | ✅ published for all device nodes; ⚠ docs call byte 3 "Virtual Interface State" (single), DBC splits into 7 interface bits + overall — DBC canonical, not a bug |
| 0xB0 RelayCommand (VCU→MCM) | ➖ relay-control FEATURE audit deferred; ⚠ **Task 5 does add fault-state gating + watchdog-feed suppression** on the shared dispatch path (per Codex review) — a RelayCommand in FAIL_HARD now returns ErrorStatus(6) and does not reset the watchdog |
| 0xB1 RelayCommandResponse (MCM→VCU) | ⚠ Task 5 fixes response to iterate subsystems (was `_primary`-only); relay signal semantics unaudited |
| 0x400 ConfigGetSet (VCU→MCM) | ✅ decoded per device; ⚠ accepts any section/param without validation (**Task 4 adds validation + ErrorStatus emission**) |
| 0x401 ConfigGetSetResponse (MCM→VCU) | ✅ emitted per subsystem |
| 0x600 IdentifyCommand (VCU→MCM) | ✅ 0-byte broadcast handled |
| 0x601/0x602/0x603 IdentifyResponse* (MCM→VCU) | ✅ all three emitted; ⚠ docs don't mention SubsystemID in Ident responses but DBC has it — DBC canonical |
| 0x700 ReportData (MCM→VCU) | ⚠ uses non-documented signal IDs 0x70–0x74. **Task 7: replace with docs-listed IDs.** 📋 docs imply reporting is opt-in per signal via ConfigGetSet Section 0x10 (or 31 per DBC) — always-on broadcast is a divergence |

### 4. Control-in-fault Enforcement (`states.md:9`, `mcm-can-api-guide.md:37`)

> "Control messages are only acted upon if the device isn't in an override OR fault state."

- ⚠ Current sim accepts ControlEnable/ControlCommand in any state and emits the normal response. **Task 5** gates them behind state checks and emits `ErrorStatus` Error_Type = 6 (State Error) when rejected. In FAIL_HARD with no control enabled, docs reference Error_Type 0x0C (Override) — we use 0x06 (State) since the sim has no override.

### 5. Watchdog Timeout (`README.md:68`, `mcm-can-api-guide.md` implicit, `simulator.py:945`)

- ⚠ Sim default is 2000 ms; docs imply 200 ms via faults-and-fault-behavior.md fault 0x30 "configured timeout." **Task 2: change default to 200 ms**; keep flag as escape hatch.

### 6. Config Section IDs (`mcm-can-api-guide.md:313`, `Error.dbc:36`)

- ⚠ Docs table says `0x10 Report`, DBC says `31 Report` (and omits Fail Op 1, Fail Op 2, Virtual Interface, Cascade). Per user decision, **DBC wins**; note in this audit only.
- ⚠ Docs reserve 0x07 and 0x08 in Error_Type; DBC uses them for "Feedback Type Error" and "Count Error". DBC canonical.

### 7. Report Signals (`mcm-can-api-guide.md:271`, `simulator.py:518`)

Current sim list (`_REPORT_SIGNALS`):

| Sim ID | Sim Label | In docs? |
|--------|-----------|----------|
| 112 (0x70) | Wheel Speed 0 | ❌ not listed |
| 113 (0x71) | Wheel Speed 1 | ❌ |
| 114 (0x72) | Wheel Speed 2 | ❌ |
| 115 (0x73) | Wheel Speed 3 | ❌ |
| 116 (0x74) | Wheel Speed Avg | ❌ |
| 80 (0x50) | Closed Loop Feedback Raw | ✅ |
| 82 (0x52) | Closed Loop Setpoint | ✅ |
| 64 (0x40) | Override State | ✅ |

**Task 7:** replace wheel-speed entries with documented signals 0x80 (Signal Input), 0x81 (Signal Output Actual), 0x83 (Signal Output Target). Keep the three already-documented ones.

### 8. Seed Rate / Fault-clear Timing (`mcm-can-api-guide.md:247`, `simulator.py:589`)

- ⚠ Docs call the rate "seed rate" and imply configurability via `Heartbeat` section (0x07). Sim hardcodes 500 ms. 📋 catalogued.

### 9. Multi-Device Concerns (`simulator.py:295`)

- ⚠ `_handle_can_message` filters by `self._args.bus_address` only — frames addressed to other devices are silently dropped. 📋 catalogued as multi-device gap, out of scope for this plan.
- ⚠ `_fault_clear_seed_loop` and `_handle_heartbeat_clear_key` only iterate primary-device subsystems. 📋 catalogued.

### 10. Out-of-Scope Doc Pages

- `signal-generation.md`, `closed-loop-control.md`, `relay-control.md` — user-deferred
- OSCC API (0x70–0x93 brake/steering/throttle) — 📋 entirely absent from sim; docs describe as legacy/optional
- `canfigurator.md`, `sygnal-config.md`, `gamepad-controller.md`, `firmware-updates.md`, `useful-can-tools.md` — external tooling, not simulator surface
- `installation/*`, `hardware/*`, `safety-documentation/*` — hardware / installation, not simulator surface

---

## Progress Tracking

- [x] Task 1: Expand `SystemState` enum to match DBC / docs (add FAIL_OPERATIONAL_1=241, FAIL_OPERATIONAL_2=242, HUMAN_OVERRIDE=253)
- [x] Task 2: Change `--watchdog-timeout-ms` default 2000 → 200; audit test fixtures
- [x] Task 3: `FaultState` (0x20) publishes only when state ∈ {FAIL_HARD, FAIL_OPERATIONAL_1, FAIL_OPERATIONAL_2, HUMAN_OVERRIDE}
- [x] Task 4: Wire up `ErrorStatus` (0x30) on CRC mismatch + unknown section / parameter / interface ID in ConfigGetSet
- [x] Task 5: Reject `ControlEnable` / `ControlCommand` / `RelayCommand` in FAIL_HARD → emit `ErrorStatus` (Error_Type 6) instead of normal response; responses iterate all subsystems; rejected frames don't feed watchdog
- [x] Task 6: Implement `FaultList` (0x220) + `FaultRootCause` (0x221) publishers, co-located with seed publisher, at seed rate while any sub in fault
- [x] Task 7: Replace `_REPORT_SIGNALS` wheel-speed entries with documented Signal IDs 0x80 / 0x81 / 0x83
- [x] Task 8: CI hooks — extend Unix socket with `set-state` / `inject-fault` / `emit-error` / `query-detailed` for deterministic CI testing of interfacing applications

**Total Tasks:** 8 | **Completed:** 8 | **Remaining:** 0

## Implementation Tasks

### Task 1: Expand SystemState enum to DBC / docs parity

**Objective:** Add the three documented state values (FAIL_OPERATIONAL_1=241, FAIL_OPERATIONAL_2=242, HUMAN_OVERRIDE=253) to the Python `SystemState` enum so downstream code (Task 3, 6) can reference them by name. No transition logic is added — these states are never entered by the simulator.
**Dependencies:** None
**Mapped Scenarios:** None (enum-level; unit-test verified)

**Files:**
- Modify: `mcm_simulator/simulator.py` (SystemState enum at line 49)
- Test: `tests/test_simulator.py` (add enum membership test)

**Key Decisions / Notes:**
- Keep it `IntEnum`; add new members between `MCM_CONTROL` and `FAIL_HARD` ordered by value.
- Do NOT add `transition()` paths into these states — docs-compliance audit marks them 📋.
- Heartbeat.dbc already uses these values in its VAL_TABLE_ — verify with `cantools` decode test.

**Definition of Done:**
- [ ] `SystemState` enum defines exactly 6 members with values 0, 1, 241, 242, 253, 254
- [ ] New enum values round-trip correctly through `Heartbeat.dbc` decode_message (unit test)
- [ ] All existing tests pass
- [ ] `ruff check .` clean

**Verify:**
- `uv run pytest tests/test_simulator.py -q`
- `uv run pytest -q` (full suite)

---

### Task 2: Watchdog default 200 ms with tightened async timing + jitter regression test

**Objective:** Align `--watchdog-timeout-ms` default with the documented real-MCM value (200 ms). Because the current asyncio timing model (100 ms recv timeout + 50 ms watchdog check tick) leaves only ~50 ms of slack under 200 ms, a naive default swap will cause false FAIL_HARD trips under normal scheduler jitter (concrete risk raised in Codex adversarial review). So this task also tightens the async timing knobs and adds a jitter regression test proving the new default holds.
**Dependencies:** None
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` — `_parse_args` (line 945), `_can_receive_loop` (line 766), `_watchdog_loop` (line 750)
- Modify: `README.md` — CLI Options table default + note
- Test: new `tests/test_watchdog_jitter.py` — regression test for no-spurious-trip under normal scheduler jitter

**Key Decisions / Notes:**
- Reduce `_can_receive_loop` executor recv timeout from `0.1` to `0.02` (20 ms).
- Reduce `_watchdog_loop` `check_interval` from `0.050` to `0.010` (10 ms).
- New timing slack = 200 − (20 + 10) = 170 ms, which is plenty for asyncio scheduler jitter.
- Argparse default → `200`. Keep the flag so local dev can raise to 2000 ms.
- Update the comment at `simulator.py:943`: explain that 200 ms matches real MCM per firmware v2.1.1 docs, and that the tightened recv/check intervals make this reliable in asyncio.
- README table shows `200` default; add a paragraph noting the escape-hatch flag.
- The implementer must `Grep` for `watchdog_timeout_ms=2000` and literal `2000` across `tests/`. Any test that NEEDS 2000 ms explicitly passes it via fixture; the rest inherit the new default.
- **Jitter regression test (`test_watchdog_jitter.py`):** launch a `SubsystemState` with 200 ms timeout, loop for 1000 ms simulating a 50 Hz heartbeat feed (20 ms gap between feeds) under the normal asyncio event loop (not mocked), and assert the sub never transitions to FAIL_HARD. Use `pytest.mark.asyncio` or `asyncio.run`. This proves the default is safe under CI-like jitter.

**Definition of Done:**
- [ ] Argparse default is 200
- [ ] `_can_receive_loop` recv timeout is 0.02 s
- [ ] `_watchdog_loop` check interval is 0.010 s
- [ ] New `test_watchdog_jitter.py` test passes: 1000 ms run with 20 ms feed gap never trips FAIL_HARD
- [ ] README table shows `200` default and escape-hatch note
- [ ] No test breakage; tests needing 2000 ms pass it explicitly
- [ ] Full pytest suite passes (baseline delta only)
- [ ] `ruff check .` clean

**Verify:**
- `uv run pytest tests/test_watchdog_jitter.py -q`
- `uv run pytest -q`
- `grep -rn "watchdog_timeout_ms=2000\|\b2000\b" mcm_simulator/ tests/` — each remaining hit is deliberate

---

### Task 3: FaultState (0x20) only emits when state is a fault state

**Objective:** Honour the docs rule: `FaultState` is only emitted while the subsystem is in a fault state. Stop emitting it every heartbeat cycle when the subsystem is in `HUMAN_CONTROL` or `MCM_CONTROL`.
**Dependencies:** Task 1 (enum has the fault-state values)
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` (`_publish_all_heartbeats` line 395, `_publish_subsystem_fault_state` line 431)
- Modify: `tests/test_diagnostics_frames.py` (update assertions)
- Test: add new test `test_fault_state_not_published_when_nominal`

**Key Decisions / Notes:**
- Gate the call at the *caller* site (`_publish_all_heartbeats`), NOT inside `_publish_subsystem_fault_state`. Rationale: existing unit tests (e.g. `test_publish_fault_state_encodes_correctly`, `test_fault_state_human_control_has_zero_cause`) call `_publish_subsystem_fault_state(sub)` directly to test the encoder. Gating at the encoder would break those tests and force rewriting them. Gating at the caller preserves the encoder-unit-test pattern and only changes the dispatch policy.
- Pattern — define a module-level helper `_is_fault_state(state: SystemState) -> bool` and reuse in Task 6 (seed loop also needs this gate).
- `test_fault_state_human_control_has_zero_cause` still works because it calls the encoder directly. Add a NEW test that exercises `_publish_all_heartbeats` with a HUMAN_CONTROL sub and asserts zero FaultState frames are emitted.

**Definition of Done:**
- [ ] `_publish_all_heartbeats` only calls `_publish_subsystem_fault_state(sub)` when `_is_fault_state(sub.state)` is true
- [ ] New test asserts: running `_publish_all_heartbeats` with a HUMAN_CONTROL sub emits zero FaultState frames
- [ ] New test asserts: running `_publish_all_heartbeats` with a FAIL_HARD sub emits exactly one FaultState frame with the correct cause
- [ ] Existing encoder-level tests (`test_publish_fault_state_encodes_correctly`, `test_fault_state_human_control_has_zero_cause`) still pass unchanged
- [ ] Full suite passes (baseline deltas only — see Risks)

**Verify:**
- `uv run pytest tests/test_diagnostics_frames.py -q`
- `uv run pytest -q`

---

### Task 4: Wire up ErrorStatus (0x30) emission

**Objective:** The `_publish_error_status` method at `simulator.py:487` is dead code. Wire it up per docs: emit `ErrorStatus` on CRC mismatch (Error_Type 1), on unknown Section ID in ConfigGetSet (Error_Type 2), on invalid Interface ID (Error_Type 3), on unknown Parameter ID (Error_Type 4).
**Dependencies:** None
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` — `_handle_can_message` (line 775) to emit 0x30 on CRC fail; `_handle_config_command` (line 606) to emit on missing section / param / interface
- Test: add new test `test_error_status_emitted_on_crc_mismatch`
- Test: add new test `test_error_status_emitted_on_invalid_section_id`

**Key Decisions / Notes:**
- DBC canonical: `ErrorStatus` encodes BusAddress, SubsystemID, Config_SectionID, Config_InterfaceID, Error_Type, Error_CANID (16-bit), CRC. Sim must pass `Error_CANID` = the offending frame's arbitration_id.
- When CRC fails before decode, we don't know SubsystemID — pass 0 and use primary device's bus_address; note this in a comment.
- When `_handle_config_command` can't find a matching device node, currently returns silently; change to emit ErrorStatus with Error_Type 1 (CRC Error) only if we had CRC-OK data but no device; Error_Type 2 when section ID is out of VAL_TABLE range.
- For invalid section/interface/param, we can detect by catching the `cantools` decode exception, OR by explicit range checks. Prefer explicit checks for `Config_SectionID > 0x1F` → section error; `Config_InterfaceID > 6` → interface error.

**Definition of Done:**
- [ ] CRC-mismatched frame triggers ErrorStatus emission with Error_Type=1
- [ ] Unknown-section ConfigGetSet (SectionID 0x1E for instance) triggers Error_Type=2
- [ ] InterfaceID > 6 triggers Error_Type=3
- [ ] Unknown Parameter ID triggers Error_Type=4
- [ ] `_publish_error_status` is no longer dead code (verified by grep)
- [ ] Unit tests cover all four error types
- [ ] Full suite passes

**Verify:**
- `uv run pytest tests/test_simulator.py::test_error_status_emitted_on_crc_mismatch -v`
- `uv run pytest -q`

---

### Task 5: Reject Control + Relay messages in FAIL_HARD; per-subsystem responses; no watchdog feed for rejected frames

**Objective:** Docs: "Control messages are only acted upon if the device isn't in an override OR fault state." Implement: when any subsystem is in FAIL_HARD, incoming ControlEnable (0x60), ControlCommand (0x160), AND RelayCommand (0xB0) must all produce an ErrorStatus (Error_Type = 6 "State Error") instead of their normal response. Also fix: responses currently emit from `_primary` only — iterate subsystems. Finally: rejected frames must NOT feed the watchdog, so that a VCU spamming a faulted subsystem with command traffic can't keep the sim's liveness timer alive.
**Dependencies:** Task 4 (ErrorStatus wiring)
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` — `_send_control_enable_response` (line 327), `_send_control_command_response` (line 343), `_send_relay_command_response` (line 359), dispatcher (line 775) including the `feed_watchdog` block at line 807
- Modify: `tests/test_simulator.py` — update any existing ControlCommand-in-FAIL_HARD tests
- Test: add `test_control_command_in_fail_hard_emits_error_status`
- Test: add `test_relay_command_in_fail_hard_emits_error_status`
- Test: add `test_control_enable_response_emitted_per_subsystem`
- Test: add `test_rejected_frame_does_not_feed_watchdog`

**Key Decisions / Notes:**
- **Per-doc protocol fact:** ControlEnable (0x60) and ControlCommand (0x160) have NO SubsystemID field in their frames — only a BusAddress. They are device-wide, so fanning out to all subsystems of the addressed device IS correct. The bug is that RESPONSES (0x61, 0x161) currently hardcode `_primary.subsystem_id` — they must iterate subsystems and emit one response per sub. Same for RelayCommandResponse (0xB1).
- **Fault gate:** in `_handle_can_message`, BEFORE calling `feed_watchdog` for the frame, check if arbitration_id ∈ {ControlEnable, ControlCommand, RelayCommand} AND any subsystem is in FAIL_HARD. If yes: emit ErrorStatus (Error_Type=6) once per faulted subsystem, SKIP the watchdog feed, and SKIP the normal response handler. This closes the loophole Codex flagged where relay traffic keeps the watchdog alive in fault states.
- **Why Relay is included despite relay-control.md being out of scope:** Codex correctly identified that the RELAY DISPATCH PATH is shared with Control in `_handle_can_message` — gating only Control while leaving Relay open means a spamming VCU can still feed the watchdog via RelayCommand and receive positive ACKs while the sim is faulted. This is a protocol correctness concern on a dispatch path we're already modifying, distinct from the deferred "Signal generation + closed-loop + relay control" feature audit.
- **Watchdog-feed scope:** rejected frames (fault-gated) don't feed watchdog; CRC-failed frames already don't reach the feed point (returned early). Other frames (Heartbeat, ConfigGetSet, HeartbeatClearKey, Identify) retain current behavior.
- Preserve `self._args.bus_address` in responses; the bus_address field identifies the responding device, not the subsystem.

**Definition of Done:**
- [ ] ControlEnable in FAIL_HARD produces one ErrorStatus per sub (Error_Type=6), zero ControlEnableResponse frames
- [ ] ControlCommand in FAIL_HARD produces one ErrorStatus per sub, zero ControlCommandResponse frames
- [ ] RelayCommand in FAIL_HARD produces one ErrorStatus per sub, zero RelayCommandResponse frames
- [ ] In HUMAN_CONTROL/MCM_CONTROL, each of the three commands produces one response PER SUBSYSTEM (not one total)
- [ ] Rejected frame paths do NOT call `sub.feed_watchdog()` — unit test asserts `sub.last_rx_time` unchanged after reject
- [ ] Tests cover all three commands × two state conditions + fan-out + watchdog-feed suppression
- [ ] Full suite passes (baseline delta only)

**Verify:**
- `uv run pytest tests/test_simulator.py -q`
- `uv run pytest -q`

---

### Task 6: FaultList (0x220) + FaultRootCause (0x221) publishers at seed rate

**Objective:** Docs say these two diagnostic frames are emitted at the seed rate while in an active fault state. Currently the CAN IDs are defined but publishers are absent.
**Dependencies:** Task 1 (enum for fault-state predicate), Task 3 (`is_fault_state` helper)
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` — add `_publish_fault_list` + `_publish_fault_root_cause`; call both from `_fault_clear_seed_loop` (consider renaming to `_fault_seed_loop`) alongside the existing seed publisher
- Test: `tests/test_diagnostics_frames.py` — add `test_fault_list_published_while_fault_active` and `test_fault_root_cause_published_while_fault_active`

**Key Decisions / Notes:**
- `FaultList` (0x220) emits one frame per non-zero fault counter. Currently the sim tracks a single `fault_count` + `fault_cause` per subsystem — that means we emit at most one FaultList frame per sub per tick. Docs say "Multiple messages will be sent out if more than one fault cause has a non-zero count" — our single-counter model emits one. Note this limitation in a code comment; don't expand the model.
- `FaultRootCause` (0x221) per DBC has `FailOp1Cause`, `FailOp2Cause`, `FailHardCause` bytes. Fill `FailHardCause` with `sub.fault_cause` when `sub.state == FAIL_HARD`; fill FailOp1/2 with 0 since sim never enters those states.
- Call both publishers inside the existing `_fault_clear_seed_loop` (after the existing seed publish, still gated on fault state). Keep the 500 ms interval; note as the "seed rate."
- In FaultList: skip emission if `fault_count == 0`.

**Definition of Done:**
- [ ] `_publish_fault_list` emits correctly-encoded 0x220 frame (cantools round-trip)
- [ ] `_publish_fault_root_cause` emits correctly-encoded 0x221 frame
- [ ] Both called only when sub is in a fault state
- [ ] No emission when sub is in HUMAN_CONTROL / MCM_CONTROL
- [ ] Both tests pass; full suite passes

**Verify:**
- `uv run pytest tests/test_diagnostics_frames.py -q`
- `uv run pytest -q`

---

### Task 7: Replace report-signal wheel-speeds with documented Signal IDs

**Objective:** `_REPORT_SIGNALS` contains 0x70–0x74 "Wheel Speed" entries that don't appear in the docs' signal table. Replace with documented IDs 0x80 (Signal Input), 0x81 (Signal Output Actual), 0x83 (Signal Output Target).
**Dependencies:** None
**Mapped Scenarios:** None

**Files:**
- Modify: `mcm_simulator/simulator.py` — `_REPORT_SIGNALS` (line 518), `_synthetic_value` (line 550)
- Modify: `README.md` — if it lists report signals, update the list
- Test: new `tests/test_signal_reporting.py` asserting (a) exactly these 6 signal IDs are broadcast, (b) each frame round-trips through the ReportData DBC

**Key Decisions / Notes:**
- Final list: 0x80 Signal Input, 0x81 Signal Output Actual, 0x83 Signal Output Target, 0x50 Feedback Raw, 0x52 Closed Loop Setpoint, 0x40 Override State. All present in docs.
- `_synthetic_value` mapping: input / output_actual / output_target — sine wave ∈ [0, 4096] (matching 12-bit ADC scale from signal-reporting.md:87); feedback raw / setpoint — same sine at different phase; override state — constant 0.
- Do NOT implement per-signal rate config (docs' Section 0x10 / 31 Report rate) — catalogued in audit, not implemented.
- Keep synthetic broadcasts always-on for now (existing behavior); add TODO comment referencing the audit's "always-on broadcast is a divergence" note.

**Definition of Done:**
- [ ] `_REPORT_SIGNALS` contains exactly the six documented Signal IDs
- [ ] `_synthetic_value` returns plausible float32 values for each
- [ ] Test asserts the exact set of Signal IDs emitted on one report tick
- [ ] Test decodes emitted frame through `ReportData` DBC successfully
- [ ] `ruff check .` clean
- [ ] Full suite passes

**Verify:**
- `uv run pytest tests/test_signal_reporting.py -q`
- `uv run pytest -q`

---

### Task 8: CI hooks — Unix socket commands for error / state / fault injection

**Objective:** (Added per user feedback at approval gate.) Extend the existing Unix socket protocol with deterministic injection commands so CI tests for interfacing applications (sygnal_can_interface_ros2_node, gamepad-controller, others) can drive the simulator into specific MCM error states / fault conditions / error frame emissions and verify the application's reaction. Without these hooks, CI can only observe organic state transitions (HUMAN_CONTROL ↔ MCM_CONTROL ↔ FAIL_HARD-via-watchdog), which is insufficient to cover all documented MCM behaviors — especially the doc-defined states the sim never enters organically (FAIL_OPERATIONAL_1, FAIL_OPERATIONAL_2, HUMAN_OVERRIDE).
**Dependencies:** Task 1 (expanded SystemState enum), Task 4 (ErrorStatus emission helper), Task 6 (FaultList/FaultRootCause publishers)
**Mapped Scenarios:** None (CI-facing protocol; tested via socket round-trip)

**Files:**
- Modify: `mcm_simulator/simulator.py` — `_handle_socket_client` (line 831) — add new command branches
- Modify: `README.md` — document the new socket protocol commands
- Test: new `tests/test_ci_hooks.py` — exercise each new command + failure modes

**Key Decisions / Notes:**
- Stay on the existing line-oriented Unix socket — no new transport — for backwards compatibility with current `estop-press` / `fail-sub*` / `recover-sub*` / `query` consumers.
- New commands (whitespace-separated args, line-terminated, `ok\n` on success, `error: <reason>\n` on failure):
  - `set-state <subsystem-id> <state-name>` — force a sub into a specific state, bypassing organic transitions. State names: `HUMAN_CONTROL`, `MCM_CONTROL`, `FAIL_HARD`, `FAIL_OP_1`, `FAIL_OP_2`, `HUMAN_OVERRIDE`. Triggers transition() so all the normal side effects (FaultIncrement emission on fault entry, fault_cause reset on clear) fire.
  - `inject-fault <subsystem-id> <fault-cause-id>` — set `fault_cause` to the given documented cause (0x01–0x42 per `faults-and-fault-behavior.md`), increment `fault_count`, and emit a one-shot FaultIncrement frame. Does NOT change state — that's a separate `set-state` call. Useful for testing how interfacing apps react to a specific fault cause without forcing a state change.
  - `emit-error <subsystem-id> <error-type> <can-id>` — emit a single one-shot ErrorStatus (0x30) frame with the given Error_Type (0x01–0x0C per docs) and Error_CANID. Does not change subsystem state. Lets CI verify ErrorStatus decoding in interfacing apps.
  - `query-detailed` — extend `query` to return JSON with per-subsystem fields: `state`, `state_int`, `fault_count`, `fault_cause`, `last_rx_age_ms`, `counter` (heartbeat counter), `estop_latched`. Existing `query` command unchanged.
- Strict input validation: unknown state name → `error: unknown state '<x>'`; invalid sub ID → `error: subsystem <n> not configured`; invalid fault cause / error type → `error: <field> out of range`. Tests cover at least one failure path per command.
- This task is the only deliberate scope expansion beyond the original "catalog gaps, implement none" directive — but it is NOT implementing a missing protocol feature; it is extending the EXISTING simulator-only test surface (Unix socket) to make the simulator genuinely useful as a CI fixture. Document this distinction in the audit section.
- Update § Docs Compliance Audit § entry for `set-state HUMAN_OVERRIDE` etc. — note that while the SIMULATOR doesn't transition into FAIL_OP_1/2 or HUMAN_OVERRIDE organically (📋 catalogued gap), CI can now force those states for testing purposes.

**Definition of Done:**
- [ ] `set-state` works for all 6 SystemState names; transitions trigger the same side effects as organic transitions
- [ ] `inject-fault` updates `fault_count` + `fault_cause` and emits exactly one FaultIncrement frame on the bus
- [ ] `emit-error` emits exactly one ErrorStatus (0x30) frame with the requested fields, decodable via Error.dbc
- [ ] `query-detailed` returns valid JSON with all documented fields per subsystem
- [ ] Existing socket commands (estop-press/release, fail-sub0/1, recover-sub0/1, query) unchanged and still pass their tests
- [ ] Each new command has at least one failure-mode test (bad sub ID, bad enum, out-of-range)
- [ ] README has a "CI Hooks" subsection under "Run" documenting each new command with example usage
- [ ] Full suite passes (baseline delta only)
- [ ] `ruff check .` clean

**Verify:**
- `uv run pytest tests/test_ci_hooks.py -q`
- `uv run pytest -q`

---

## Codex Adversarial Review — Resolved

All four Codex findings (3 high + 1 medium, verdict "needs-attention") incorporated:

1. **DBC-canonical framing refined** → § Context for Implementer now states sygnal_docs v2.1.1 = source of truth; DBC = byte-layout authority for shared messages; missing-from-DBC == catalogued gap.
2. **Task 5 per-subsystem routing clarified** → docs confirm ControlEnable/Command have no SubsystemID in the request (device-wide); the bug is in response emission (hardcoded `_primary`). Task 5 now explicitly fixes response fan-out for all three commands.
3. **Relay included in shared dispatch fix** → Task 5 now gates RelayCommand on fault state AND suppresses watchdog feed for rejected frames, closing the loophole where relay traffic kept the sim "alive" in FAIL_HARD. Scope note added to § Out of Scope clarifying this is a shared-path correctness fix, not a relay-control.md audit.
4. **Watchdog 200 ms safeguarded** → Task 2 expanded to also tighten recv timeout (100 → 20 ms) and check interval (50 → 10 ms); new `test_watchdog_jitter.py` regression test proves the default holds under normal scheduler jitter.

## Open Questions

*(none remaining — scope, canonical source, bug-fix set, audit location, report signal handling, fidelity policy, and Codex findings all resolved)*

### Deferred Ideas

- Full Human Override state implementation (requires modeling analog input signals)
- FAIL_OPERATIONAL_1/2 transition logic driven by per-fault-source count thresholds
- Inbound FaultState (0x20) handler → increments local fault counter per `faults-and-fault-behavior.md` fault IDs 0x03–0x05
- Override Status (0x40) + Override Acknowledge (0x41) message pair
- OSCC API (0x70–0x93 brake/steering/throttle legacy protocol)
- Per-signal configurable report rates (ConfigGetSet Section 0x10/31)
- Configurable "seed rate" (currently hardcoded 500 ms)
- Multi-device receive routing (ControlCommand/Enable for non-primary devices silently dropped)
- Input/output signal bound validation → fault cause 0x20/0x21
- Control Count8 validation → fault cause 0x31/0x33
- Control value bounds → fault cause 0x34

# Dexhand servo bus bring-up and control

Bring-up, diagnostics and a PySide6 control GUI for the 8-DoF Dexhand prototype: Feetech
SCS0009 and HLS3606 servos, mixed per servo ID on one TTL half-duplex bus, driven from macOS or
Windows through a Feetech FE-URT-2 over USB, either directly or through an ESP32-S3 bridge (see
*ESP32-S3 bridge*). See *Servo models* for what differs between the two servo models.

The hand has **no mechanical end stops**. Every packet is checked twice against the active write
policy, in [safe_bus.py](dexhand_servo/safe_bus.py):

1. In the SDK's `txPacket`, before the SDK frames the packet.
2. On the final bytes in `writePort`, just before they reach the serial port.

A refused packet raises `UnsafeOperationError`, and nothing reaches the wire.

| Command | Policy | What may be written |
|---|---|---|
| `diagnose`, `monitor` | read-only | nothing (PING / READ only) |
| `assign-id` | id-assignment | one byte to register 5 (ID) or the model's EEPROM lock (48 SCS0009, 55 HLS3606) |
| `verify-motion`, `gui` | motion ([control/policy.py](dexhand_servo/control/policy.py)) | see below |
| `tune-servo` | servo-tuning ([tune_servo.py](dexhand_servo/tune_servo.py)) | one byte to register 21 (P gain; SCS0009 5..60, HLS3606 16..64), 24 (min start force, 0..100) or the model's lock, on one servo; torque must be off |
| `hls-setup` | hls-setup ([hls_setup.py](dexhand_servo/hls_setup.py)) | HLS3606 only, one servo: angle limits 9/11 (one 4-byte WRITE), position offset 31, lock 55; torque must be off |

The motion policy allows only:

- **torque off**, for any configured ID;
- **torque on**, for joints armed through the arming sequence;
- **goal blocks**, single WRITE or SYNC_WRITE, under all of these conditions:
  - the joint is armed (which first verifies the servo against its `JointConfig`, see *Servo models*);
  - the position is inside that joint's configured travel;
  - SCS0009: goal time is 0; HLS3606: goal current (register 44) is 1..the cap verified at arming,
    and no word has its sign bit set;
  - 1 ≤ speed ≤ the joint's speed cap;
  - in the GUI, the goal moves at most `max_speed × 0.1 s + 2` steps from the previously accepted
    goal (per-packet jump limit). The only exception is a single-write pose (see *Single-write
    poses*): the controller pre-authorises exactly that position and speed, the speed may not exceed
    the joint's `max_speed`, and the authorisation is used up by the one packet that carries it.

  Speed 0 means "maximum speed" on SCS servos ("stop" on the HLS) and is always refused.

Everything else is refused: EEPROM writes, the lock register, REG_WRITE/ACTION, reset, and
broadcast WRITE. A SYNC_WRITE with one bad entry is refused as a whole. (The ESP32 bridge's link
watchdog sends one packet of its own, a broadcast torque off; see *ESP32-S3 bridge*.)

**Bus census.** An HLS3606 also acts on WRITE / SYNC_WRITE packets addressed to its *secondary* ID
(register 7; factory default 0), which PING and READ cannot reveal. Before anything is written,
every write command PINGs IDs 0–253 and reads registers 0–7 of each responder (about 1–5 s).
Until that census succeeds, every write except torque-off is refused. After it, a write to an ID
that is another servo's secondary ID is refused. Torque-off is always allowed, so an E-stop is never
blocked. The census fails, and writes stay refused, if a responder cannot be read cleanly or is
not a supported model.

## Setup

Requires [uv](https://docs.astral.sh/uv/getting-started/installation/). uv reads
`.python-version` and installs Python 3.12 by itself if it is missing.

```sh
git clone <repo-url> servo_bringup
cd servo_bringup
uv sync --locked --extra gui   # drop --extra gui if you only need the CLI tools
uv run pytest                  # offline: simulated servos and an offscreen GUI, no hardware needed
uv run dexhand-servo list-ports
```

Never copy a `.venv` between machines: it contains absolute paths and platform-specific binaries.
Delete it and run `uv sync` again.

Without uv, use the pinned, hash-checked requirements exported from `uv.lock`:

```sh
python -m venv .venv                         # Python 3.12+
source .venv/bin/activate                    # Windows: .venv\Scripts\activate
pip install --require-hashes -r requirements-dev.txt   # or requirements.txt: no pytest
pip install --no-deps -e .
```

Serial ports: macOS uses `/dev/cu.*` (the tool maps `/dev/tty.*` to `cu.*` for you); Windows uses
`COMn`, e.g. `--port COM5`. `list-ports` shows what is connected.

## Collaborating

- `uv.lock` is committed and is the source of truth for dependency versions. Commit it whenever it
  changes.
- Add or change a dependency with `uv add <pkg>` (or `uv add --dev <pkg>`), never by editing
  `requirements*.txt` by hand. Then regenerate the requirements files:

  ```sh
  uv export --locked --extra gui --no-dev --no-emit-project -o requirements.txt
  uv export --locked --extra gui --no-emit-project -o requirements-dev.txt
  ```

- Run `uv run pytest` before pushing. The suite needs no hardware, and must pass on both macOS and
  Windows.
- Use `time.perf_counter()` for timing, not `time.monotonic()`: on Windows with Python 3.12,
  `monotonic()` only ticks every 15.6 ms.
- Open text files with `encoding="utf-8"`; the Windows default (cp1252) cannot write the plot
  pages.
- Line endings are normalised to LF by `.gitattributes`.
- `reports/` (run output) is ignored. `data/` (poses, recordings, original servo tuning) is
  tracked, because it is tied to this specific hand.
- **[config.py](dexhand_servo/config.py) `JOINTS` is safety-critical:** the travel limits are the
  only thing preventing a joint from being driven past its mechanical range. Review changes to it
  as carefully as code changes, and say in the commit message how the limits were measured.

## ESP32-S3 bridge

Instead of the URT-2's own USB port, the PC can reach the bus through an ESP32-S3-DevKitC-1
(N32R16V) running [firmware/urt_bridge](firmware/urt_bridge):

```
PC --USB (DevKit "USB" port)--> ESP32-S3 --UART1--> URT-2 TTL input --> servo bus
```

The bridge is transparent: bytes pass unchanged both ways, and the UART follows the baud rate the
PC sets (only the rates in `SUPPORTED_BAUDRATES`), so every command, the guard, the census and the
`diagnose` baud sweep work as before. `list-ports` shows it as `ESP32 bridge`. When both the bridge
and the URT-2's USB port are plugged in, pass `--port`: the tool never guesses between them.

**Wiring.** GPIO17 (TX) → URT-2 RX, GPIO18 (RX) ← URT-2 TX, GND ↔ GND. Set the URT-2 to take the
bus from its TTL/MCU header instead of its USB chip (see the board's silkscreen / jumper). Check
the URT-2's TTL level first: the ESP32 is a 3.3 V part, so a 5 V TX line from the URT-2 needs a
divider before GPIO18. GPIO33–37 are used by the module's octal flash/PSRAM and cannot be used.

**Power the ESP32 independently of the PC link** (DevKit "UART" port on a USB charger, or its 5V
pin from a separate supply). If the PC's USB cable is its only supply, unplugging that cable
switches the ESP32 off and the watchdog cannot act; the servos then keep holding their last goal.

**Build and flash** ([PlatformIO](https://platformio.org/), from `firmware/urt_bridge`):

```sh
pio test -e native                                   # host unit tests, no hardware
pio run -e esp32s3 -t upload --upload-port <DevKit "UART" port>
pio device monitor -p <DevKit "UART" port>           # bridge log, 115200 baud
```

The log port is UART0; the PC tool uses the native "USB" port. The RGB LED pin differs by board
revision (`RGB_LED_PIN` in `platformio.ini`: 48 on v1.0, 38 on v1.1).

**Link watchdog.** The bridge parses (never alters) the PC's packets and tracks which IDs may hold
torque: a WRITE / SYNC_WRITE that sets register 40 non-zero or touches the goal block 42–47 adds
the ID, a write of register 40 = 0 removes it. While any ID is tracked, the bridge sends torque off
when

- no byte has come from the PC for 500 ms (`WATCHDOG_MS`; the GUI polls at 50 Hz, and
  `verify-motion` pauses at most 0.3 s with torque on), or
- the PC closes the port or its process dies (DTR drops), or USB is suspended.

It waits for 2 ms of bus silence, finishes any half-sent PC packet with a deliberately wrong
checksum (so the servos discard it), then sends the broadcast WRITE "register 40 = 0" three
times. LED: dim blue = idle, green = torque tracked, red = watchdog fired (until the PC sends
again). A clean shutdown of the GUI or a CLI command switches torque off itself, so the watchdog
stays quiet. The broadcast only reaches servos at the bus's current baud rate.

## Pre-flight hardware checklist (before first power-up)

1. Multimeter on both soldered harnesses: no short between VCC / GND / SIG; continuity from every
   servo lead to its terminal.
2. Both 3-pin terminals match the URT-2 header silkscreen (GND / VCC / SIG). Reversed polarity can
   destroy the servos.
3. URT-2 set to the TTL servo bus (if the board has a TTL/485 selector).
4. Servo supply 5 V, preferably a bench supply with current limit (~1 A). Plug in USB first, then
   servo power.
5. Rest the hand with every joint free (nothing pressed against a hard surface). This is an extra
   safeguard on top of the software guard.
6. With the ESP32 bridge: TTL levels checked (3.3 V on GPIO18), ESP32 powered independently of the
   PC link, and `diagnose` S1/S2 passing through the bridge before any write.

## Why IDs must be assigned first

All SCS0009 ship with ID 1. With all eight on one bus, every request to ID 1 is answered by eight
servos at once, so their replies overlap. Any write to ID 1 would also change all eight. Software
cannot tell them apart, so each servo has to be connected to the bus on its own once, to give it a
unique ID.

Harness rework: keep VCC and GND soldered together, and bring each servo's **SIG** lead out to its
own 2.54 mm jumper / Dupont connector, so signal lines can be connected one at a time.

## Operator procedure

| Step | Setup | Command | Expected |
|---|---|---|---|
| 1 | USB only, servo power **off** | `uv run dexhand-servo diagnose --ids 1-8` | S1/S2 pass, scan finds nothing, exit 2. Baseline: no false positives. |
| 2 | Current harness, power on | `uv run dexhand-servo diagnose` | ID 1 = `SUSPECTED_ID_CONFLICT`, exit 1. Proves the bus link works and confirms the ID clash. |
| 3 | Reworked harness: assigned servos + **exactly one** un-assigned servo on SIG | `uv run dexhand-servo assign-id --to K` for K = 2…8 | Each run verifies, asks you to type K, writes, re-verifies. The last remaining servo keeps ID 1. Then fill `JOINTS` in [config.py](dexhand_servo/config.py) (see *Joint map and travel limits*). |
| 4 | All connected, power-cycled | `uv run dexhand-servo diagnose --ids 1-8` | 8 × `HEALTHY_SINGLE`, register dumps, exit 0. |

Every run writes a JSON report to `reports/`, including raw TX/RX hex for every exchange that did
not come back OK. Pass `--all-traffic` to record every exchange.

## What `diagnose` does

| Stage | Purpose |
|---|---|
| S0 port discovery | Prefers WCH (VID 0x1A86) adapters and uses `/dev/cu.*`. If there are several candidates, pass `--port`. |
| S1 idle line | Listens 200 ms without transmitting. Any bytes received mean noise, a wiring fault or a second master. |
| S2 echo | PINGs unused ID 253. If the adapter echoes TX, the SDK would treat the echo as a servo reply, so the run aborts. |
| S3 ID scan | PINGs 0–253 (or `--ids`), with 2 retries per ID. `--baud-sweep` tries all 8 SCS baud rates if nothing answers at 1 Mbps. |
| S4 link assessment | Per ID: reads of the present-state block, ID-register reads, then 100 PINGs. Each ID gets one of four verdicts (below). |
| S5 registers | Only for `HEALTHY_SINGLE`: identifies the model from registers 0–4, reads its memory table in chunks (0–70 SCS0009, 0–86 HLS3606), decodes it with that model's map and byte order, and runs sanity checks: model vs `JOINTS`, ID, baud, voltage, temperature, torque off, position within angle limits, status, moving flag; for the HLS also multi-turn mode (limits 0/0), operating mode, angle resolution and position offset. A fleet check fails when an HLS secondary ID equals another servo's or joint's ID. |

S4 verdicts:

| Verdict | Meaning |
|---|---|
| `HEALTHY_SINGLE` | Clean, consistent replies from one servo. |
| `SUSPECTED_ID_CONFLICT` | Corrupt frames, or an ID register that does not match the addressed ID. Position spread on clean frames is only a sanity-check warning: overlapping replies with differing positions cannot all pass the checksum. |
| `UNRELIABLE` | Timeouts but no corruption. Usually power or wiring. |
| `NO_RESPONSE` | No clean reply during the assessment. |

Reads come before PINGs in S4 because servos sharing an ID can return identical PING replies that
overlap cleanly. Their positions differ, so read replies come out corrupt. A conflicted ID is
dropped after 5 corrupt frames, which keeps the time several servos drive the line at once short.

Exit codes: 0 = all healthy, 1 = link up but conflicts or warnings, 2 = no link / aborted.

## `monitor`: live position / voltage capture

Read-only. Samples the present-state block (position, speed, load, voltage, temperature) of each
ID once per cycle, shows a live table with a trace of the last 40 samples, and on exit (end of
`--duration`, or Ctrl-C) writes `reports/monitor_<timestamp>.csv` with every sample and a
self-contained `.html` chart page: position and supply voltage over time for each servo, with a
hover crosshair, light and dark themes.

```sh
uv run dexhand-servo monitor --ids 2 --duration 30          # feedback noise on one servo
uv run dexhand-servo monitor --ids 1-8 --rate 50            # whole hand, 10 s
uv run dexhand-servo monitor --ids 5 --duration 0           # until Ctrl-C, e.g. while sweeping a joint by hand
```

| Column | Meaning |
|---|---|
| Spread | max - min position, in steps and degrees (nominal 300 deg / 1024 steps) |
| Max step | largest change between two consecutive clean samples |
| Jumps>N | consecutive changes larger than `--jump-threshold` (default 5 steps) |
| V | min-max supply voltage reported by the servo |
| Fail | samples without a clean reply |

Typical uses:

- **Feedback noise** (hand at rest, torque off): spread should be about 0. Jumps that line up with
  voltage dips point to the supply. Jumps that change when you touch the servo lead point to wiring
  or a worn potentiometer.
- **Joint range**: run with `--duration 0`, slowly move the joint by hand from one mechanical stop
  to the other, then Ctrl-C. Min/max are the candidates for that joint's software angle limits.
  Keep a few steps of margin from both stops.

`--rate` is in cycles per second, and each cycle reads every listed ID. Each read takes about
0.5 ms at 1 Mbps, so 8 IDs at 50 Hz is comfortable. If the requested rate cannot be reached, the
tool reports the rate it actually achieved.

## Joint map and travel limits

[config.py](dexhand_servo/config.py) `JOINTS` is the only table edited by hand. Each entry maps a
servo ID to the joint it drives and to that joint's allowed travel, in raw servo steps
(0–1023, nominal 0.293 deg/step):

```python
JOINTS: dict[int, JointConfig] = {
    1: JointConfig("thumb_cmc", min_position=118, max_position=642),
    2: JointConfig("TBD"),                      # not identified / not measured yet
    ...
}
```

| Field | Meaning |
|---|---|
| key | servo ID (1–253) as assigned with `assign-id` |
| `name` | unique joint name; `"TBD"` until identified |
| `min_position`, `max_position` | allowed travel **including** safety margin; both set or both `None` |

**1. Identify which ID drives which joint.** Torque is off, so every joint can be moved by hand.
Run `uv run dexhand-servo monitor --ids 1-8 --duration 0` and move one joint at a time. The ID whose
`Pos` follows your hand is that joint's servo. Write its name into the entry and repeat for all
eight joints.

**2. Measure each joint's mechanical travel.** For each ID:

1. Run `uv run dexhand-servo monitor --ids K --duration 0`.
2. Move the joint slowly from one mechanical stop to the other. Stop at the real end of travel:
   tendon fully taut, or finger fully extended against its structure. Don't force it.
3. Do two or three passes, then press Ctrl-C. `Min` / `Max` in the table, and the labelled
   extremes in the HTML chart, are the raw stops.

**3. Apply a margin.** `min_position = Min + margin`, `max_position = Max - margin`.

- Start with about 15 steps (≈ 4.4 deg) per side.
- Use more if the position noise seen in `monitor` at rest is large (spread > 5 steps), or if the
  joint is compliant (tendon stretch).
- The servo has no hard stop of its own, so the margin is the only buffer against overshoot.

**4. Check.** Run `uv run dexhand-servo diagnose --ids 1-8`. Every command first validates
`JOINTS` (IDs 1–253, unique names, both limits set, `0 <= min < max <= 1023`) and refuses to run
on inconsistent entries. `diagnose` then warns when:

- the resting position lies outside the configured travel (wrong limits, wrong mapping, or a
  mis-assembled joint),
- the configured travel exceeds the servo's EEPROM angle limits (registers 9/11),
- a servo answers but has no `JOINTS` entry or no limits yet.

`monitor` shows the configured travel per ID and marks `OUT` when the current position is outside it.

These limits are enforced at packet level by the motion policy: a goal outside
`[min_position, max_position]` never reaches the servo, whichever code built it. A joint without
limits cannot be armed. `max_speed` (default 150 steps/s) caps every generated trajectory for that
joint.

## Servo models (SCS0009 and HLS3606)

Each `JOINTS` entry names its servo model; the default is `"scs0009"`. Both models speak the same
packet protocol, so they share one bus, and one SYNC_WRITE can carry goal blocks for both, each
encoded for its servo. Everything else is per model ([models.py](dexhand_servo/models.py)):

| | SCS0009 | HLS3606 |
|---|---|---|
| 16-bit byte order (register 2, END) | big-endian (1) | little-endian (0) |
| Position | 0–1023 over ~300°, 0.293°/step | 0–4095 per turn, 0.088°/step; BIT15 = sign in multi-turn mode |
| Speed register | ~steps/s (load-dependent), 0 = maximum | 0.732 RPM ≈ 50 steps/s per unit, 0 = stop; the servo's speed loop really limits speed |
| Register 44 in the goal block | goal time (always 0) | goal current, 6.5 mA per unit |
| EEPROM lock | 48 | 55 (48 is the torque limit) |
| Secondary ID | none | register 7: the servo also obeys WRITEs to this ID |
| Current reading | none | register 69 (mA) |

```python
JOINTS = {
    1: JointConfig("triple_pip", min_position=350, max_position=730),          # SCS0009
    6: JointConfig("thumb_mcp_rot", min_position=1700, max_position=2400, max_speed=500,
                   model="hls3606", current_limit_ma=2000, position_offset=-2044),
}
```

| Field | Meaning |
|---|---|
| `model` | `"scs0009"` (default) or `"hls3606"`; checked against the servo before every arming |
| `min_position`, `max_position`, `max_speed` | in that servo's own steps and steps/s (HLS: 0–4095) |
| `current_limit_ma` | HLS only: cap for the goal current; `None` uses the servo's protection current (register 28, 5.2 A on the first unit) |
| `position_offset` | HLS only: the EEPROM position offset (register 31) the travel was measured with |

`MotionConfig` thresholds (tolerances, jog steps, tracking / stall / mismatch limits, acceleration,
arming and streaming speeds) and the `verify-motion` arguments are written in SCS0009 steps and are
converted per joint, so an HLS joint gets the same angles and angular speeds. `DEFAULT_MAX_SPEED`
(150 steps/s) is therefore slow on an HLS joint (~13°/s); set `max_speed` from
`verify-motion --speed-sweep`.

**Arming an HLS joint** additionally requires, read from the servo each time: single-turn mode
with EEPROM angle limits that contain the configured travel, operating mode 0, angle resolution 1,
the configured position offset, a non-negative position, and a valid protection current. The goal
current sent with every goal is `min(protection current, current_limit_ma)`.

### HLS3606 bring-up

HLS servos may leave the factory in multi-turn mode (angle limits 0/0): the servo then does not
limit travel, and with full-angle feedback the position can go negative after crossing zero.

1. Assign a unique ID with `assign-id` as for the SCS0009 (the lock is register 55).
2. `uv run dexhand-servo diagnose --ids K`: model HLS3606, multi-turn warning expected.
3. Optional, recommended: `uv run dexhand-servo hls-setup --id K --center` with the joint at rest
   mid-travel. Writes the position offset that puts the present position at 2048, far from the
   0/4095 seam (refused once the joint has travel limits in `JOINTS`). Power-cycle.
4. Measure the travel with `monitor` (in HLS steps), enter it in `JOINTS` with
   `model="hls3606"` and `position_offset=` the value written in step 3.
5. `uv run dexhand-servo hls-setup --id K --limits-from-config`: writes that travel as the EEPROM
   angle limits (both in one 4-byte WRITE), which also leaves multi-turn mode. Power-cycle.
6. `diagnose --ids K` must be clean, then `verify-motion --id K`, then the GUI.

`hls-setup --show` prints limits, offset, mode and position; `--restore` writes back the values
saved in `data/servo_tuning/id<K>_hls_original.json` before the first change.

## Driving the hand: `verify-motion` and `gui`

### Before the first motion

1. Have a **physical servo power cut-off** within reach. Use a current-limited supply if you can.
   SCS0009 servos have no communication watchdog: if USB drops or the program is killed, an armed
   servo keeps holding its last goal with torque on. Only the servo's own overload cut-out (80 %
   for 1 s) still acts.
2. `diagnose --ids 1-8` must be clean. The supply window is 5.5–6.5 V
   ([config.py](dexhand_servo/config.py) `SUPPLY_VOLTAGE_WINDOW`).
3. Run `uv run dexhand-servo verify-motion --id K` on at least one joint. It moves only that joint,
   ±20 steps around its present position, and asks for confirmation before every step:

| Check | Why the driver cares |
|---|---|
| A. does a goal write switch torque on by itself? | The controller never sends a goal to a disarmed joint, and reads before it writes, so it cannot re-enable a servo that dropped torque. This check shows how much that matters on this firmware. |
| B. does torque-on alone hold the position? | Arming writes goal = present first, then torque. |
| C. speed register response, load sign, which direction is flexion | On the real hand the register turned out **not** to be a velocity: at register 50 the joints moved at 1.6–55 steps/s depending on direction and load. See *Streaming* below. |
| D. SYNC_WRITE for goals and for torque off | The control loop and the E-stop both use it. |
| E. (`--overload-test`, you hold the finger) does the servo's own cut-out release torque, and what do registers 40/65 read? | Tells you whether the supervisor will see such an event. |
| F. (`--speed-sweep`) actual speed in both directions at registers 100 / 300 / 600 / 1000 | Prints a suggested `max_speed` for the joint: 70 % of the slower direction at register 1000, which is the register the GUI streams with. If the joint does not reach the ±20-step target, it is reported as **force-limited** and no speed is suggested: its creep rate is not a speed limit (see *Force-limited joints*). |
| G. (`--single-write`, `--single-write-steps 40`) one goal write per direction at registers 100 and 300, sampled as fast as the bus allows | Shows how the servo executes a long move on its own: start latency, ramp time (internal acceleration, or a jump to full speed), steady speed vs register, overshoot, settling time, start-up load. It decides whether a pose could be sent as a single SYNC_WRITE instead of streamed. Also writes `reports/verify_motion_id<K>_*_single_write.html`. |

The report is saved to `reports/verify_motion_id<K>_*.json`.

### Running the GUI

```sh
uv run dexhand-servo gui              # real hand (port auto-detected, or --port)
uv run dexhand-servo gui --rate 100   # control loop at 100 Hz instead of MotionConfig.cycle_hz
uv run dexhand-servo gui --stream-speed fixed   # compare against the old fixed speed register
uv run dexhand-servo gui --simulate   # simulated servos, for trying the UI without hardware
```

The top bar shows the measured loop rate and the time one cycle takes against its budget
(`cycle 5.1/10 ms`). The log warns if a cycle keeps using more than 80 % of its budget; if it
does, lower the rate.

Every pose, sync and playback move is traced. The log prints, per joint, when it arrived (stayed
within ±3 steps of its goal), the spread of arrival times, and the worst tracking error.
`reports/trace_<time>_<label>.html` plots setpoint vs actual position and load for each joint;
the matching `.csv` holds the raw samples, including the speed register sent each cycle. Two
smoothness figures appear per joint:

- **max goal step**: the largest change of the sent goal between two cycles (stair height);
- **stalled cycles**: the share of cycles in which the goal advanced ≥ 1 step but the joint did not move.

Compare them between `--rate 50/100` and `--stream-speed dynamic/fixed`.

On startup every joint is switched to torque off. Nothing moves until you arm a joint.

**Incomplete servo sets.** Joints that do not reply at startup are shown as **OFFLINE**: they are
not polled, do not block the other joints, are left out of the torque-off confirmation and cannot
be armed, so a single servo can be debugged on its own. After connecting more servos, press
**Rescan bus** (enabled only while no joint is armed): it reads every configured joint again,
switches newly found ones to torque off and redoes the HLS secondary-ID census. A joint that
stops replying *after* it was online is still treated as a communication loss (arming blocked;
all joints torque off if any is armed). Arming still needs a `JOINTS` entry with travel limits and
a matching servo model.

| Area | Content |
|---|---|
| Top bar | link and loop rate (and how many joints are offline), mode selector, Arm all / Torque off all / Stop motion / Reset faults / Rescan bus, **E-STOP** |
| Banner | latched E-stop or fault, lost communication, unconfirmed torque-off ("cut servo power"), moves in progress |
| Joint panels (4×2) | arm/disarm button, travel bar (white dot = actual, blue triangle = target), target slider, position (steps/°), target, **load %**, voltage, temperature, comm health, faults and warnings |
| Mode page | slider · poses · sync move · jog · teach/playback |
| Log | every event; also written to `reports/session_*.log` |

**Load and current:** the SCS0009 has no usable current register (69–70 always read 0), so the
panels show load (drive duty in %, register 60). HLS3606 panels also show the phase current
(register 69, mA). Positions are shown in each servo's own steps, with degrees per model.

**E-stop:**
- the red button, **Esc** (always), or **Space** (except while typing in a text field);
- closing the window, Ctrl-C or SIGTERM;
- automatic trips (below).

It disarms everything, sends a SYNC_WRITE torque-off, then per-ID torque-off writes, and reads
back to confirm. It also discards queued commands. It stays latched until **Reset faults**, and a
reset never re-arms a joint.

### Modes

| Mode | Use |
|---|---|
| Slider | drag a joint's slider; the target is clamped to its travel; the mouse wheel is ignored |
| Poses | move all armed joints smoothly and synchronously to a saved pose (`data/poses.json`); built-in `center` = mid-travel. With **Single write** ticked, *Go to pose* sends the pose as one SYNC_WRITE instead (below) |
| Sync move | set targets for several joints; they start and arrive together |
| Jog | nudge one joint by 1 / 5 / 10 steps (hold a button to repeat) |
| Teach / playback | record with torque off, moving the hand by hand. Playback (joints armed) first approaches frame 0 smoothly, then follows the keyframes at ×0.1–1.0 speed. Recordings go to `data/recordings/` |

All motion goes through the same per-joint limiter (`JointConfig.max_speed`, default
150 steps/s, plus `MotionConfig.acceleration`). Pose and recording files store the ID → joint map
and are refused if the map or the travel limits no longer match `config.py`.

### Streaming (how setpoints reach the servos)

- **Every cycle.** One SYNC_WRITE carries a new goal for each armed joint whose torque was just
  read as on.
- **Planned moves.** A move's duration respects every joint's `max_speed` and 90 % of
  `MotionConfig.acceleration`. All joints follow the same minimum-jerk curve, so they start and
  arrive together. The limiter feeds the target velocity forward, so it follows a feasible
  trajectory exactly instead of lagging by v²/(2a) steps. It only reshapes infeasible targets,
  such as a slider dragged far at once.
- **Speed register.** In `dynamic` mode (default, `MotionConfig.stream_speed_mode`), each goal
  is sent with speed register = max(`stream_speed_floor` 50, 1.3 × the joint's current setpoint
  velocity), capped at 1000. On freely moving joints the register is close to steps/s
  (`verify-motion --speed-sweep`), so the servo's internal reference glides from one goal update
  to the next instead of jumping there and waiting, which removes the per-cycle stop-go.
  `fixed` mode always sends 1000, the earlier behaviour. Speed is never the safety limit: the
  trajectory and the motion policy's per-packet jump limit are.
- **Tuning.** Set each joint's `max_speed` from `verify-motion --speed-sweep`, so that a planned
  duration is something every joint can really achieve.

### Single-write poses

The **Single write** option on the Poses page sends the whole pose in **one** SYNC_WRITE: each
armed joint gets its final goal with speed register = its `max_speed`, and the servos pace the move
themselves. This is what `verify-motion --single-write` characterises; on ID 1 the servo started
50–100 ms after the write, jumped straight to full speed (no ramp), and overshot 5–6 steps at
register 300.

- The packet is sent in the send phase of a cycle, only if every joint in it was just read with
  torque on. It passes the motion policy through a one-time authorisation (exact position and
  speed, speed ≤ `max_speed`); every other packet still obeys the jump limit.
- Joints are not synchronised: each one arrives after distance / `max_speed`. Force-limited joints
  may not reach the goal at all (see *Force-limited joints*).
- While the move is in flight nothing is streamed to its joints, and the supervisor pauses the
  tracking-error, stall and parked-under-load checks for them. Load, temperature, supply,
  communication, torque state, servo errors and goal-register mismatch stay active.
- It ends when every joint is within 3 steps of its goal, or after
  `2 × expected duration + 1 s`. On timeout the log lists the joints that did not arrive. In both
  cases streaming resumes at the sent goal, and normal supervision resumes too, so a joint still
  far from its goal trips on tracking error.
- **Stop motion**, a mode change, disarming one of its joints or a late GUI heartbeat stops it. The
  remaining joints get one more authorised SYNC_WRITE: goal = the position just read, speed 50,
  so they hold where they are. A new move is refused until it has ended.
- Traces work as for streamed moves (`stream_speed_mode` = `single write`).

### Automatic protection ([control/supervisor.py](dexhand_servo/control/supervisor.py), thresholds in `MotionConfig`)

| Condition | Default | Action |
|---|---|---|
| Load (interference) | > 60 % for 0.3 s, or > 75 % for 0.1 s | torque off that joint |
| Tracking error | > 25 steps for 0.4 s | torque off that joint |
| Stall | setpoint moving, joint not moving, load > 40 % for 0.3 s | torque off that joint |
| Goal register not following the sent setpoint | > 3 steps for 0.3 s | torque off that joint |
| Torque state mismatch (servo dropped torque, or torque on while not armed) | immediate | **all** off |
| Servo error byte or status | immediate | that joint (all for a voltage error) |
| Communication | 3 failed reads in a row | **all** off; arming blocked |
| Supply | median outside 5.5–6.5 V for 0.2 s | **all** off; arming blocked |
| Temperature | ≥ 65 °C for 1 s (warning at 55 °C) | that joint |
| HLS3606 current at its goal-current cap | ≥ 90 % of the cap for 0.2 s (`current_trip_frac`, `current_trip_s`) | that joint |
| GUI heartbeat | > 0.5 s: motion frozen · > 2 s | **all** off |
| Parked under load: setpoint still, ≥ 3 steps short, load ≥ 25 % | for 3 s | warning in the panel and the log (the servo keeps pushing and heats up); no automatic stop |

Every trip is latched and shown in the joint panel or the banner. Arming is refused until you
press **Reset faults**.

### Force-limited joints and `tune-servo`

The SCS0009 position loop is proportional-only: its output is about 1.5–1.7 % load per step of
error (measured on this hand, P gain 15). A joint whose friction or tendon preload needs more
force than that:

- lags until the error has built up, then jumps (stick-slip);
- stops short of its goal;
- keeps that load on indefinitely.

The fix is mechanical: less friction or preload. If a residual error remains after that, raise
the servo's gain on that joint only:

```sh
uv run dexhand-servo tune-servo --id 3 --show
uv run dexhand-servo tune-servo --id 3 --p-gain 25                  # factory 15; allowed 5..60
uv run dexhand-servo tune-servo --id 3 --min-start 20               # register 24; allowed 0..100 raw
uv run dexhand-servo tune-servo --id 3 --restore                    # original values back
```

How it works:

- Only registers 21 (P gain) and 24 (minimum start force) can be written, plus the EEPROM lock.
  The packet-level `TuningPolicy` enforces this, and the value ranges.
- The servo must have torque off, and you type its ID to confirm.
- Before the first change, the values are saved to `data/servo_tuning/id<K>_original.json`.
  Every change is also logged in `changes.jsonl`.

Raise P in steps of ≤ 10. After each step, arm the joint in the GUI, jog it, and watch for
buzzing or oscillation.

### Suggested hardware bring-up order

1. `gui` with nothing armed: telemetry should match `monitor`. Press E-STOP: the log should say
   "torque off confirmed".
2. `verify-motion --id K` on one joint.
3. GUI: arm one joint and jog it slowly towards each end of its travel, watching the load.
4. Trip tests with one joint armed:
   - hold the finger;
   - unplug USB while it is holding (with the ESP32 bridge: also kill the GUI process; the joint
     should go limp within ~0.5 s and the LED turn red);
   - lower the supply to 5.4 V.
5. All joints at low speed, then poses, sync moves and playback. Raise `max_speed` per joint only
   after that.

## Caveats

- The SCS0009 register map in [registers.py](dexhand_servo/registers.py) comes from a third-party
  SCSCL table and the SDK constants. Decoded values are indicative only. The raw bytes are always
  stored, so check them against the official SCS0009 manual. The HLS3606 map follows the official
  Feetech table ([docs/hls-memory-table.md](docs/hls-memory-table.md)).
- 16-bit values are decoded big-endian on the SCS0009 and little-endian on the HLS3606; the model
  (and so the byte order) is identified from registers 0–4 (END flag, servo version).
- The HLS manual does not say whether EEPROM angle limits and the position offset apply without a
  power cycle, nor the sign convention of the offset; `hls-setup` reads everything back and asks
  for a power cycle followed by `diagnose`.
- The URT-2's USB-serial chip is not documented publicly. If `list-ports` shows nothing, install
  the chip vendor's driver (WCH CH34x is built into recent macOS; on Windows, install WCH's CH341SER
  driver if Device Manager shows an unknown USB device).
- After `assign-id`, power-cycle and re-run `diagnose` to confirm the ID persisted in EEPROM.
- The ESP32 bridge adds USB-to-UART forwarding time to every exchange. Compare `diagnose` latency
  p50/p99 through the bridge with a direct URT-2 run before relying on the GUI's
  `MotionConfig.control_latency_ms` (4 ms); raise it if replies time out.
- The bridge is USB only for now. A WiFi link needs batched bus exchanges: a 50 Hz cycle makes
  about ten request/reply round trips, which a per-packet WiFi bridge cannot fit.

# turtle_recovery

Turtle mode (crash-flip recovery) for Quad-X multicopters running DShot ESCs.

Based on *PX4 Turtle Recovery* by Yisrael Ben-Shlomo
(<https://github.com/BENSHLOMO/px4-fpv-extensions>, BSD-3-Clause), generalised
(no Dragonfly-specific naming, legacy helpers removed) for the Formic fork.

## What it does

While **disarmed**, with `TRTL_EN=1`, `TRTL_ESC=1` and the switch on `TRTL_CH`
held ON while the vehicle is tilted more than `TRTL_ANGLE` from upright, the
DShot driver:

1. stops all motors (`TRTL_PRE`),
2. sends DShot command 21 (temporary reverse) to the four mapped outputs,
3. waits `TRTL_SETTLE`,
4. drives the motor pair / corner selected by the roll/pitch stick for at most
   `TRTL_TIME`,
5. stops (`TRTL_POST`), sends DShot command 20 (temporary normal), cools down,
6. and waits for the switch to return OFF before releasing the outputs.

The same stop/restore row runs once at boot so ESCs are normalised before the
first arm. Commander refuses to arm while recovery owns the outputs
(`ArmingInterlock.hpp`).

DShot 20/21 are *relative* to the ESC's configured direction and volatile, so
no direction state is stored; the ESC never acknowledges them.

## Files

| File | Role |
| --- | --- |
| `TurtleCore.hpp` | State machine (IDLE → HOLD → PRESTOP → REVERSE → SETTLE → DRIVE → POSTSTOP → RESTORE → COOLDOWN → WAIT_OFF) |
| `TurtleMapping.hpp` | Corner ↔ logical motor ↔ output bit mapping, stick → motor selection |
| `TurtleHelpers.hpp` | Freshness, attitude verticality, RC health/selection, geometry sanity |
| `ArmingInterlock.hpp` | FLIGHT / RECOVERY output ownership shared by Commander and the DShot driver |
| `test/` | Host unit tests: `test/run_host_tests.sh` |

Hook points: `src/drivers/dshot/DShot.cpp` (`updateOutputs()`, `Run()`,
`update_turtle_parameters()`, `update_turtle_inputs()`),
`src/drivers/dshot/module.yaml` (`TRTL_*`), `src/modules/commander/Commander.cpp`
(`arm()` / `disarm()` guards), `src/lib/mixer_module/mixer_module.hpp`
(`isActuatorTestRunning()`).

## Before flying

Validated upstream only on AM32 ESCs. For any other firmware (e.g. OX32),
confirm props-off from NSH that `dshot reverse -m N` / `dshot normal -m N`
reverse and restore direction, then set `TRTL_FL/FR/RL/RR`, `TRTL_ESC=1`,
`TRTL_EN=1` and check `dshot status`.

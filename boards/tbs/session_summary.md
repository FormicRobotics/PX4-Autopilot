# TBS Lucid H7 Porting — Session Summary

Summary of the Claude Code session working on the `boards/tbs/lucid_h7` PX4 port,
for reference before starting the next round of work.

## 1. Cross-checked the planning doc against the actual code

`clude_data.md` (ArduPilot hwdef-derived pinout/planning notes) still says
"no PX4 board files exist yet — planning material only," but the board is
actually already substantially implemented (SPI, timers, defconfig, board_config.h,
etc.). Most of it matches the doc cleanly:

- SPI: IMU1 on SPI1/PC15 CS, IMU2 on SPI4/PE11 CS — matches.
- PWM/DShot: TIM3/TIM2/TIM5/TIM4/TIM15/TIM1 channel-to-pin mapping matches the
  doc's 13-channel table exactly (incl. TIM1_CH1/PA8 for the WS2812 strip).
- UART order, I2C, CAN1, SD card, ADC channels all line up with the doc.

## 2. Pin conflicts found and fixed in `src/board_config.h`

Several defines looked like leftover boilerplate from a generic FMU template,
never adapted to this board's real pinout, and collided with pins already
assigned to real peripherals in the same file:

| Define (removed/changed) | Pin | Conflicted with |
|---|---|---|
| `GPIO_TONE_ALARM` (TIM17_CH1) | PB9 | UART4_TX — SERIAL4 Telem1/MAVLink2 |
| `GPIO_HEATER_OUTPUT` | PA8 | TIM1_CH1 — WS2812 LED (PWM13) |
| `GPIO_OTGFS_VBUS` | PA9 | USART1_TX — SERIAL1 DJI SBUS in |
| `GPIO_BTN_SAFETY` | PC15 | IMU1_CS |
| `GPIO_nSAFETY_SWITCH_LED_OUT` | PB1 | TIM3_CH4 — PWM2 motor output |
| `GPIO_CAN2_RX/TX` in `PX4_GPIO_INIT_LIST` | — | board only has one CAN bus (CAN1) |

Fixes applied:

- Tone alarm moved to a plain GPIO buzzer on **PA15** (matches ArduPilot hwdef:
  `PA15 BUZZER OUTPUT`) using `GPIO_TONE_ALARM_IDLE` / `GPIO_TONE_ALARM_GPIO`
  (no timer needed — same pattern as `holybro/kakuteh7mini`).
- Removed `GPIO_HEATER_OUTPUT` — no heater on this board per the hwdef.
- Removed `GPIO_OTGFS_VBUS` / `BOARD_ADC_USB_CONNECTED` — no verified VBUS-sense
  pin exists in the hwdef; PA9 is genuinely a UART pin, not VBUS. Removing this
  also required dropping `CONFIG_DRIVERS_CDCACM_AUTOSTART` and
  `CONFIG_SYSTEMCMDS_USB_CONNECTED` from `default.px4board` since both hard-require
  `board_read_VBUS_state()`, which only compiles when `GPIO_OTGFS_VBUS` is defined
  (`platforms/nuttx/src/px4/common/board_ctrl.c`). This was confirmed by an actual
  link failure (`undefined reference to board_read_VBUS_state`) and fixed.
- Removed safety button/switch (`GPIO_BTN_SAFETY`, `GPIO_nSAFETY_SWITCH_LED_OUT*`,
  `GPIO_LED_SAFETY`) and `CONFIG_DRIVERS_SAFETY_BUTTON` — no physical safety
  switch on this board (matches `kakuteh7mini`/`kakuteh7dualimu` pattern).

**Note:** at one point the whole `lucid_h7` board tree reverted to its original
unfixed state mid-session (git history/timestamps suggest the environment reset
or the user regenerated the files) and all of the above had to be reapplied a
second time. Worth checking this is durable/committed once verified.

## 3. IMU driver fix: ICM42688 → ICM42688P (user's catch)

The board was wired for the plain `ICM42688` devtype/driver, but PX4 ships two
separate IMU drivers for this chip family — `icm42688` and `icm42688p` — and
every comparable board (`kakuteh7mini`, `kakuteh7dualimu`, `matek/h743-slim`)
uses the **P** variant. Fixed in three places:

- `default.px4board`: `CONFIG_DRIVERS_IMU_INVENSENSE_ICM42688P=y`
- `src/spi.cpp`: both `initSPIDevice(...)` calls now use `DRV_IMU_DEVTYPE_ICM42688P`
- `init/rc.board_sensors`: driver invocations changed from `icm42688 -s -b ...`
  to `icm42688p -s -b ...`

## 4. Build environment note

The very first build attempt used `sudo make tbs_lucid_h7_default`, which left
`build/tbs_lucid_h7_default/` root-owned. All subsequent non-sudo builds failed
with `Permission denied` on CMake configure — unrelated to the code changes.
Fix: `sudo chown -R naor:naor build/tbs_lucid_h7_default` (or `sudo rm -rf` it)
once, then always build **without** `sudo` from then on.

## 5. Forum thread investigated: DShot boot failure / DMA question

Found and read the user's own thread,
[Porting TBS Lucid H7](https://discuss.px4.io/t/porting-tbs-lucid-h7/48492)
(discuss.px4.io, posted 2026‑02‑13). No answer posted there yet (2 replies,
neither about DMA). Symptom: `ERROR [dshot] module not running`,
`ERROR [commander] Timed out while waiting for thread to start`; sensors
(ICM42688P, DPS310) work fine.

### Analysis (not yet applied to code — user wants to test the fixes above first)

Two key facts about STM32H7 DMA in this codebase
(`platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/hw_description.h`):

1. `getTimerUpdateDMAMap()` / `getTimerChannelDMAMap()` only branch on
   `dma.index` (Index1→DMA1, Index2→DMA2). The `DMA::Stream`/`DMA::Channel`
   values used in `timer_config.cpp` (e.g. `Stream2, Channel5`) are **not read
   at all** on H7 — those are dead parameters left over from F4/F7-style fixed
   stream tables. The real stream is picked dynamically at runtime by NuttX's
   DMAMUX allocator, confined to whichever controller `.index` selects.
2. Each DMA controller (DMA1, DMA2) has only 8 physical streams.

Current allocation:
- `nuttx-config/include/board_dma_map.h` deliberately splits the IMUs:
  SPI1 (IMU1) → DMA1, SPI4 (IMU2) → DMA2.
- But `src/timer_config.cpp` puts **all five** DShot-capable timers (TIM3,
  TIM2, TIM5, TIM4, TIM1) on `DMA::Index1` too — so DMA1 ends up with
  2 (SPI1) + 5 (timer UP-DMA) = 7 of its 8 streams, while DMA2 sits almost
  idle (just SPI4's 2 streams).

This contradicts `clude_data.md` section 5's own original DMA plan, which
wanted the timers spread onto DMA2 specifically to avoid exactly this kind of
DMA1 contention with the IMU SPI traffic. This imbalance is the leading
suspect for the dshot-thread-timeout boot failure, though not yet proven —
the user is testing the current (non-DMA) fixes first.

**Proposed next step (not yet applied):** rebalance `timer_config.cpp` — move
roughly half the PWM timers (e.g. TIM2 and TIM4, or TIM1) from `DMA::Index1`
to `DMA::Index2` so each controller carries one IMU's SPI DMA plus 2–3 timers
instead of concentrating everything on DMA1.

## 6. Open/unresolved

- `default.px4board` changed on disk mid-session (outside this session's edits)
  and lost several modules (`MC_ATT_CONTROL`, `MC_POS_CONTROL`, `NAVIGATOR`,
  `SENSORS`, `RC_UPDATE`, `UXRCE_DDS_CLIENT`, and `CONFIG_ARCH_CHIP_STM32H7`).
  Flagged to the user as possibly accidental (especially losing
  `CONFIG_ARCH_CHIP_STM32H7`) — not reverted, not confirmed either way.
- DMA rebalancing in `timer_config.cpp` is diagnosed but intentionally **not
  yet applied** — user wants to test the current build/flash first and come
  back with results.

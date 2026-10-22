# TBS Lucid H7

## UART / serial port map

| Pads | UART | MCU pins (TX / RX) | PX4 device | PX4 port name | Intended use |
|---|---|---|---|---|---|
| TX1 / RX1 | USART1 | PA9 / PA10 | `/dev/ttyS0` | TELEM2 | DJI SBUS in |
| TX2 / RX2 | USART2 | PD5 / PD6 | `/dev/ttyS1` | GPS1 | GPS |
| TX3 / RX3 | USART3 | PD8 / PD9 | `/dev/ttyS2` | GPS2 | DJI O3 / MSP DisplayPort |
| TX4 / RX4 | UART4 | PB9 / PB8 | `/dev/ttyS3` | TELEM3 | Telem1 (MAVLink) |
| TX6 / RX6 | USART6 | PC6 / PC7 | `/dev/ttyS4` | TELEM1 | RC input |
| TX7 / RX7 | UART7 | PE8 / PE7 | `/dev/ttyS5` | RC | Telem2, HW flow control (3.3 V only, not 5 V tolerant) |
| TX8 / RX8 | UART8 | PE1 / PE0 | `/dev/ttyS6` | — | ESC telemetry |

- The board has no UART5. NuttX numbers the enabled UARTs in order
  (USART1, USART2, USART3, UART4, USART6, UART7, UART8) with no gaps, which gives
  the `/dev/ttyS` numbers above.
- "PX4 port name" comes from `default.px4board` (`CONFIG_BOARD_SERIAL_*`). It is
  the name to select in a `*_CONFIG` parameter (e.g. `MAV_1_CONFIG`, `GPS_1_CONFIG`).
  For example, to use the TX1/RX1 pads, select **TELEM 2**.

### Known issues (to verify)

- Several PX4 port names don't match the intended use (e.g. TELEM1 is the RC
  input UART).
- `RC_SERIAL_PORT` in `src/board_config.h` is `/dev/ttyS5` (UART7), but the RC
  receiver is wired to USART6 (`/dev/ttyS4`).
- The UART-to-`ttyS` comment block in `nuttx-config/include/board.h` is stale.

To confirm the mapping on hardware, run `ls /dev/tty*` in the NSH shell and use
`serial_test` to see which device receives data on a given pad.

## Camera switch

The analog camera input is selected by PD11 (HIGH = camera 1, LOW = camera 2).
PD11 is HIGH at boot, so camera 1 is the default.

To switch cameras from the radio:

1. Map a radio switch to an AUX channel, e.g. `RC_MAP_AUX1 = 7`.
2. Set `TBS_CAM_SWITCH` to that AUX number (1-6) and reboot.

`TBS_CAM_SWITCH = 0` (default) ignores RC and selects camera 1.

AUX above center selects camera 1, below center selects camera 2. If RC is lost,
the last selected camera is kept.

From the NSH shell: `camera_switch set 1|2` selects a camera manually (works even
when the module isn't running), and `camera_switch status` shows the current state.

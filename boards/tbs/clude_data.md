# TBS Lucid H7 FC — ArduPilot → PX4 Porting Reference

Status: research/planning phase. No PX4 board support exists yet for this board.
Goal: create a new PX4 board target under `boards/tbs/lucid-h7/` (or similar) using
ArduPilot's existing hwdef.dat as the pinout ground truth.

---

## 1. Hardware Summary

- **MCU:** STM32H743VIH6 (LQFP100 package), 480 MHz, 2MB flash, 1MB RAM
- **IMU:** Dual ICM42688 (some v2 boards: dual MPU6000), each on its own SPI bus
- **Baro:** DPS310 (V1) / DPS368 (V2), on I2C2
- **OSD:** AT7456E / MAX7456-compatible, on SPI2
- **Storage:** microSD via SDMMC1; external QSPI/SPI flash NOT present on this variant
  (ArduPilot hwdef uses onboard flash pages for HAL_STORAGE, not an external flash chip)
- **CAN:** 1x CAN1
- **PWM/DShot:** 13 outputs across 6 timers

Reference sources used:
- Official TBS manual (Rev 1.15): https://www.team-blacksheep.com/media/files/tbs-lucid-manual.pdf (section 6, "H7 FC")
- ArduPilot hwdef: `libraries/AP_HAL_ChibiOS/hwdef/TBS_LUCID_H7/hwdef.dat` (full text pasted below)
- ArduPilot README: `libraries/AP_HAL_ChibiOS/hwdef/TBS_LUCID_H7/README.md`
- iNav PR #11631 (cross-confirms CAN pins sourced from the same AP hwdef)

---

## 2. Full ArduPilot hwdef.dat (ground truth pinout)

```
# hw definition file for processing by chibios_pins.py
# for TBS FCAPv1 H743

MCU STM32H7xx STM32H743xx
APJ_BOARD_ID AP_HW_TBS_LUCID_H7
OSCILLATOR_HZ 8000000
FLASH_SIZE_KB 2048
MCU_CLOCKRATE_MHZ 480
FLASH_RESERVE_START_KB 128
STM32_ST_USE_TIMER 12
define CH_CFG_ST_RESOLUTION 16

# USB
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD

# SPI1 for IMU1 (ICM42688)
PA5 SPI1_SCK SPI1
PA6 SPI1_MISO SPI1
PD7 SPI1_MOSI SPI1        # NOTE: moved off PA7 on purpose - see section 4
PC15 IMU1_CS CS

# SPI2 for MAX7456 OSD
PB12 MAX7456_CS CS
PB13 SPI2_SCK SPI2
PB14 SPI2_MISO SPI2
PB15 SPI2_MOSI SPI2

# SPI3 - external (unused pin header)
PB3 SPI3_SCK SPI3
PB4 SPI3_MISO SPI3
PB5 SPI3_MOSI SPI3
PD4 EXT_CS1 CS
PE2 EXT_CS2 CS

# SPI4 for IMU2 (ICM42688)
PE11 IMU2_CS CS
PE12 SPI4_SCK SPI4
PE13 SPI4_MISO SPI4
PE14 SPI4_MOSI SPI4

# I2C
I2C_ORDER I2C2 I2C1
PB6 I2C1_SCL I2C1 PULLUP
PB7 I2C1_SDA I2C1 PULLUP
PB10 I2C2_SCL I2C2 PULLUP   # internal baro
PB11 I2C2_SDA I2C2 PULLUP

# ADC
PC0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PC1 BATT_CURRENT_SENS ADC1 SCALE(1)
PA4 BATT2_VOLTAGE_SENS ADC1 SCALE(1)
PA7 BATT2_CURRENT_SENS ADC1 SCALE(1)   # NOTE: this is why SPI1_MOSI moved to PD7
PC4 PRESSURE_SENS ADC1 SCALE(2)         # airspeed
PC5 RSSI_ADC ADC1

define HAL_BATT_MONITOR_DEFAULT 4
define HAL_BATT_VOLT_PIN 10
define HAL_BATT_CURR_PIN 11
define HAL_BATT2_VOLT_PIN 18
define HAL_BATT2_CURR_PIN 7
define HAL_BATT_VOLT_SCALE 11.0
define HAL_BATT_CURR_SCALE 40.0
define HAL_BATT2_VOLT_SCALE 11.0
define HAL_DEFAULT_AIRSPEED_PIN 4
define BOARD_RSSI_ANA_PIN 8

# LEDs
PE3 LED0 OUTPUT LOW GPIO(90)   # blue
PE4 LED1 OUTPUT LOW GPIO(91)   # green

# UARTs
SERIAL_ORDER OTG1 USART1 USART2 USART3 UART4 EMPTY USART6 UART7 UART8 OTG2

PA10 USART1_RX USART1 NODMA
PA9  USART1_TX USART1 NODMA      # SERIAL1: DJI SBUS in

PD5 USART2_TX USART2
PD6 USART2_RX USART2             # SERIAL2: GPS1

PD9 USART3_RX USART3
PD8 USART3_TX USART3             # SERIAL3: DJI O3 / MSP DisplayPort

PB9 UART4_TX UART4 NODMA
PB8 UART4_RX UART4 NODMA         # SERIAL4: Telem1, MAVLink2

PC6 USART6_TX USART6
PC7 USART6_RX USART6             # SERIAL6: RC input

PE7 UART7_RX UART7
PE8 UART7_TX UART7
PE10 UART7_CTS UART7
PE9  UART7_RTS UART7             # SERIAL7: Telem2, HW flow control, 3.3V ONLY (not 5V tolerant)

PE0 UART8_RX UART8 NODMA
PE1 UART8_TX UART8 NODMA         # SERIAL8: ESC telemetry

# CAN
PD0 CAN1_RX CAN1
PD1 CAN1_TX CAN1
PD3 GPIO_CAN1_SILENT OUTPUT PUSHPULL SPEED_LOW LOW GPIO(70)

# Motors / PWM (6 timers, 13 channels)
PB0  TIM3_CH3 TIM3 PWM(1)  GPIO(50) BIDIR
PB1  TIM3_CH4 TIM3 PWM(2)  GPIO(51)
PA0  TIM2_CH1 TIM2 PWM(3)  GPIO(52) BIDIR
PA1  TIM2_CH2 TIM2 PWM(4)  GPIO(53)
PA2  TIM5_CH3 TIM5 PWM(5)  GPIO(54) BIDIR
PA3  TIM5_CH4 TIM5 PWM(6)  GPIO(55)
PD12 TIM4_CH1 TIM4 PWM(7)  GPIO(56) BIDIR
PD13 TIM4_CH2 TIM4 PWM(8)  GPIO(57)
PD14 TIM4_CH3 TIM4 PWM(9)  GPIO(58) BIDIR
PD15 TIM4_CH4 TIM4 PWM(10) GPIO(59)
PE5  TIM15_CH1 TIM15 PWM(11) GPIO(60) NODMA
PE6  TIM15_CH2 TIM15 PWM(12) GPIO(61) NODMA
PA8  TIM1_CH1 TIM1 PWM(13) GPIO(62)   # WS2812 LED strip

# Beeper
PA15 BUZZER OUTPUT GPIO(32) LOW

# microSD
PC8 SDMMC1_D0 SDMMC1
PC9 SDMMC1_D1 SDMMC1
PC10 SDMMC1_D2 SDMMC1
PC11 SDMMC1_D3 SDMMC1
PC12 SDMMC1_CK SDMMC1
PD2 SDMMC1_CMD SDMMC1

# GPIOs (relays)
PD10 VSW OUTPUT GPIO(81) LOW     # voltage switch (RELAY2)
PD11 CAMSW OUTPUT GPIO(82) LOW   # camera switch (RELAY3)
PC13 VPWR OUTPUT GPIO(83) LOW    # 9V VTX enable (RELAY4)

# DMA hints (ChibiOS-specific, informational only for PX4 purposes)
DMA_PRIORITY SPI1* SPI4*
DMA_NOSHARE SPI1* SPI4* TIM3* TIM2* TIM5* TIM4*

# SPI device table
SPIDEV imu1 SPI1 DEVID1 IMU1_CS MODE3 2*MHZ 16*MHZ
SPIDEV imu2 SPI4 DEVID1 IMU2_CS MODE3 2*MHZ 16*MHZ
SPIDEV osd  SPI2 DEVID4 MAX7456_CS MODE0 10*MHZ 10*MHZ

IMU Invensensev3 SPI:imu1 ROTATION_YAW_270
IMU Invensensev3 SPI:imu2 ROTATION_YAW_180
BARO DPS310 I2C:0:0x76
```

---

## 3. UART Map (ArduPilot SERIALn -> PX4 /dev/ttySn candidate)

| AP Serial | UART | Pins | Function | Notes |
|---|---|---|---|---|
| SERIAL0 | USB (OTG1) | PA11/PA12 | MAVLink console | |
| SERIAL1 | USART1 | PA9/PA10 | DJI SBUS in | RX-heavy use |
| SERIAL2 | USART2 | PD5/PD6 | GPS1 | DMA-capable |
| SERIAL3 | USART3 | PD8/PD9 | DJI O3 / MSP DisplayPort | DMA-capable |
| SERIAL4 | UART4 | PB8/PB9 | Telem1 (MAVLink2) | NODMA |
| SERIAL6 | USART6 | PC6/PC7 | RC input | |
| SERIAL7 | UART7 | PE7-PE10 | Telem2, HW flow ctrl | **3.3V only, not 5V tolerant** |
| SERIAL8 | UART8 | PE0/PE1 | ESC telemetry | NODMA |

No SERIAL5 (skipped in SERIAL_ORDER — matches the physical UART numbering gap on this MCU/board combo).

---

## 4. Known Hardware Quirks to Preserve in the PX4 Port

1. **SPI1_MOSI is on PD7, not PA7.** This is deliberate: PA7 is used as an ADC pin
   (`BATT2_CURRENT_SENS`). If the PX4 pinmux is set up "by datasheet default" without
   checking this, you'll get an ADC/SPI pin conflict.
2. **Two IMUs on two independent SPI controllers** (SPI1 and SPI4) — not sharing a bus.
   This simplifies PX4 bus/CS driver setup considerably compared to boards where both
   IMUs share one SPI bus with separate CS lines.
3. **UART7 is 3.3V only** — do not wire 5V peripherals directly to it.
4. **STM32H743's DMAMUX is fully dynamic** (unlike F4/F7's fixed stream-to-peripheral
   tables). Any DMA request line can be routed to any of the 16 total DMA streams
   (DMA1: 8, DMA2: 8). This means there is no fixed "conflict table" to consult —
   the constraint is simply having enough free streams, which is not an issue here
   (need ~9, have 16).
5. **Known STM32H7 + NuttX DMA gotchas** (seen even on official PX4 fmu-v6x, which hit
   a DMA issue with TIM5_CH4 driving a WS2812 LED strip — GitHub issue #20693):
   - D-Cache coherency: DMA buffers must be placed in non-cached memory or the code
     must explicitly clean/invalidate cache lines around DMA transfers.
   - DMA1/DMA2 cannot access all SRAM domains (e.g., some ITCM/DTCM regions are
     off-limits to DMA) — buffer placement matters.
   - The DMAMUX peripheral itself needs its own RCC clock enable before use; easy to
     forget when porting from a board where this was already handled.

---

## 5. Proposed DMA Stream Assignment (draft, needs verification against RM0433)

| Consumer | Request | Proposed DMA stream |
|---|---|---|
| SPI1 (IMU1) RX | SPI1_RX | DMA1 Stream 0 |
| SPI1 (IMU1) TX | SPI1_TX | DMA1 Stream 1 |
| SPI4 (IMU2) RX | SPI4_RX | DMA1 Stream 2 |
| SPI4 (IMU2) TX | SPI4_TX | DMA1 Stream 3 |
| TIM2 (M3,M4) | TIM2_UP | DMA2 Stream 0 |
| TIM3 (M1,M2) | TIM3_UP | DMA2 Stream 1 |
| TIM4 (M7-M10) | TIM4_UP | DMA2 Stream 2 |
| TIM5 (M5,M6) | TIM5_UP | DMA2 Stream 3 |
| TIM1 (LED/WS2812) | TIM1_UP | DMA2 Stream 4 |

TIM15 (PWM 11-12) is NODMA in the ArduPilot config and can likely stay NODMA in PX4 too
(only relevant for high-rate DShot on those two channels).

---

## 6. PX4 Reference Boards Used for Comparison

- `boards/px4/fmu-v6x/src/board_config.h` — GPIO define style reference (STM32H7 family)
- `boards/px4/fmu-v6x/src/timer_config.cpp` (not yet fetched in full) — modern
  `initIOTimer(Timer::TimerX, DMA{DMA::IndexN, DMA::StreamN, DMA::ChannelN})` style
- PX4/PX4-Autopilot PR #13871 — "Board config: timer config simplifications" —
  explains the modern compact timer/DMA config format

---

## 7. Open Items / Next Steps

- [ ] Fetch `boards/px4/fmu-v6x/src/timer_config.cpp` in full for a concrete template
- [ ] Fetch `boards/px4/fmu-v6x/src/spi.cpp` for SPI bus registration template
- [ ] Verify PD7 as a valid SPI1_MOSI AF mapping in NuttX's STM32H743 pinmap
      (`nuttx/arch/arm/src/stm32h7/hardware/stm32h7x3xx_pinmap.h` or similar)
- [ ] Decide on PX4 board directory name/vendor namespace (e.g. `boards/tbs/lucid-h7/`)
- [ ] Create `default.px4board` enabling: DShot, ICM42688 driver, DPS310 baro driver,
      UAVCAN/DroneCAN, SD card logging, RGBLED (WS2812 via TIM1)
- [ ] Write `nuttx-config/` (defconfig, board.h) — largest unknown-effort item
- [ ] Bring-up order recommendation: UART console first → LEDs → IMU SPI → PWM/DShot →
      CAN → SD card (matches typical PX4 board bring-up sequence)

---

## 8. How This Document Was Built

Sourced from a conversation with Claude (Sonnet 5, Anthropic) combining:
web search of ArduPilot's GitHub hwdef, the official TBS Lucid manual PDF, PX4's
official fmu-v6x board support files, and community forum/GitHub reports of a partial,
unofficial in-progress PX4 port of this same board (discuss.px4.io thread "Porting tbs
lucid h7"). No PX4 board files exist yet — this is planning material only.

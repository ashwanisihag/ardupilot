# SaamPixV1_2

This folder contains the ChibiOS hardware definition for the SaamPixV1_2 flight
controller (Saam Drones). The design is derived from the Pixhawk6C reference.

## Build

From repository root:

```bash
./waf configure --board SaamPixV1_2
./waf copter
```

Output artifacts are generated under `build/SaamPixV1_2/bin/`.

## Flash

Build and upload:

```bash
./waf configure --board SaamPixV1_2
./waf copter --upload
```

For manual flashing, use generated firmware files in `build/SaamPixV1_2/bin/`.

## Hardware Notes

- FMU: STM32H743VIH6 — **100-pin TFBGA100**, not the 176-pin part used on
  Pixhawk6C. Port H only bonds out PH0/PH1, so any Pixhawk6C pin assignment on
  PH2 and above is invalid here.
- IOMCU: STM32F103C8T6, running stock `iofirmware_f103_lowpolh.bin` over USART6
  (FMU) / USART2 (IO), coupled through 33R series resistors.
- IMUs: MPU-9250 (U2) and ICM-20602 (IC5), both on SPI1 with independent CS and
  DRDY. No BMI055 is fitted.
- Barometer: MS5611 (U4) on I2C1 at 0x76. A DPS310 (U5) is also fitted on the
  same bus but is not enabled — see `hwdef.dat`.
- Compass: no compass is enabled by default. An on-board LIS3MDL (MAG1, 0x1C)
  exists on I2C1 but shares that bus with the GPS1 connector.
- Storage: RAMTRON FM25V02 on SPI2 (`HAL_WITH_RAMTRON`).
- Safety switch: provided by the IOMCU (`HAL_HAVE_SAFETY_SWITCH 1`).
- Spektrum/DSM power: controlled by the IOMCU (its PC13), not by the FMU.
- APJ board token: `AP_HW_SAAMPIXV1_2` (1208)
- APJ board name: `SaamPixV1_2`

USB identity is set to Saam-specific values in both `hwdef.dat` and
`hwdef-bl.dat`.

For pin mapping and buses, see `hwdef.dat` and bootloader mapping in
`hwdef-bl.dat`.

## Open hardware items

These could not be resolved from the KiCad design files and must be confirmed
against the physical board:

1. **X2 crystal frequency.** `OSCILLATOR_HZ` is set to 16 MHz (matching the
   Pixhawk6C reference and the schematic text annotation), but the X2 symbol
   carries `PARTNO "NX3225SA-24.000000MHZ"`. Both X1 and X2 share that PARTNO
   because they use the same library part, so it is likely stale — but if a
   24 MHz crystal is actually fitted, the PLL will run 1.5x over spec.
   Note X1 (IOMCU) *must* be 24 MHz for the stock IO firmware, so the "8MHZ"
   annotation next to X1 is definitely wrong. Fix the schematic PARTNO fields
   so the BOM orders the right parts.
2. **DPS310 address.** Its schematic symbol has an incorrect (EEPROM-style)
   pin-name mapping, so its I2C address and interface-mode strapping are
   unknown. If it lands on 0x76 it collides with the MS5611.
3. **IMU orientations.** `ROTATION_PITCH_180` (U2, MPU-9250) and
   `ROTATION_ROLL_180_YAW_270` (IC5, ICM-20602) are derived from the PCB pad
   geometry with vehicle forward = the -X end of the board (the USB / GPS end,
   opposite the IOMCU). The derivation is written out in `hwdef.dat`. The only
   input not taken from the design files is the InvenSense axis convention
   (top view, pin 1 upper-left, +X right, +Y up, +Z out of the package top).
   Confirm on the bench before flying.
4. **IOMCU PB1 is tied to the 3V3 rail.** The stock IO firmware configures PB1
   as `TIM3_CH4`, PWM output 8. As wired, IO channel 8 is unusable and the pin
   will short 3V3 to ground whenever the firmware drives it low. This needs a
   board rework.

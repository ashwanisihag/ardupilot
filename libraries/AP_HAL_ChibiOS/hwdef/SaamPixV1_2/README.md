# SaamPixV1_2

This folder contains the ChibiOS hardware definition for the SaamPixV1_2 flight controller.

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

- MCU: STM32H743xx
- Storage: RAMTRON enabled (`HAL_WITH_RAMTRON`)
- Internal safety switch: disabled (`HAL_HAVE_SAFETY_SWITCH 0`)
- APJ board token: `AP_HW_SAAMPIXV1_2`
- APJ board name: `SaamPixV1_2`

USB identity is set to Saam-specific values in both `hwdef.dat` and `hwdef-bl.dat`.

For pin mapping and buses, see `hwdef.dat` and bootloader mapping in `hwdef-bl.dat`.
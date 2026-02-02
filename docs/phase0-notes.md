# Phase0: Power + I2C + Regmap Bring-up

## Goal
Verify ES8388 power rails and I2C communication.

## Steps
- Enable regulators via DTS
- Setup regmap
- Sanity read register 0x00

## Result
- Probe success
- Regmap read OK
- No I2C error

## Issues
- TBD


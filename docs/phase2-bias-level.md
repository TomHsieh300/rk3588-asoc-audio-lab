# Phase2: Bias level bring-up (set_bias_level)

## Goal
Wire up and validate the ASoC set_bias_level() hook for ES8388.

## What I implemented
- Add `.set_bias_level` callback in `snd_soc_component_driver`
- Add register dump helper (R00–R04) for before/after visibility
- Implement minimal register writes for OFF / STANDBY / PREPARE / ON

## Validation
- Print bias transition logs in dmesg
- Dump key registers before/after each bias state

## Notes / TODO
- The forced bias transitions in component probe are for bring-up
  validation only and should be removed once lifecycle is verified.
- Convert raw writes to `regmap_update_bits()` where appropriate.


# Phase1: ASoC component + DAI skeleton

## Goal
Integrate ES8388 driver with ASoC framework.

## Changes
- Register snd_soc_component_driver
- Expose snd_soc_dai_driver (Playback/Capture capability declaration)

## Result
- Codec can be bound by machine driver via standard DAI link

## Next
- Phase2: implement set_fmt and hw_params

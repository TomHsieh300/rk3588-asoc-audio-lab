# RK3588 ES8388 ASoC Codec Bring-up & Tuning Lab

This repository documents a complete bring-up and tuning workflow
for the ES8388 audio codec on the RK3588 platform using Linux ASoC.

The project focuses on codec-level engineering topics including
power sequencing, DAPM routing, pop-noise suppression,
and playback latency optimization.

This is a codec-centric practice project and does not aim to
implement a full custom machine driver.

---

## Project Goals

- Independently bring up an ASoC codec driver from scratch
- Understand ES8388 internal power and bias architecture
- Implement stable DAPM routing
- Minimize pop/click noise during playback start/stop
- Optimize playback startup latency
- Build solid codec driver engineering skills

---

## Current Status

| Phase | Description                                   | Status |
|-------|-----------------------------------------------|--------|
| 0     | Power, I2C, Regmap bring-up                   | ✔      |
| 1     | ASoC component & DAI registration             | ✔      |
| 2     | Bias level & power sequencing                 | ✔      |
| 3     | DAPM routing & signal path validation         | ✔      |
| 4     | Pop-noise suppression & latency optimization  | ✔      |

---

## Key Engineering Topics

### 1. Power & Bias Control

- Custom `set_bias_level()` implementation
- VMID/VREF sequencing analysis
- Analog bias stability verification
- Power state validation via regmap debugfs

### 2. DAPM Routing

- Complete playback signal path construction
- Widget dependency analysis
- Route stability tuning

### 3. Pop-Noise Suppression

- Stream-level mute control
- DAC digital volume soft ramp (R1A/R1B)
- Reduced reliance on DAPM settle delays
- Near-vendor-level noise performance

### 4. Playback Latency Optimization

- Removal of redundant DAPM delays
- Faster playback startup
- Stable unmute timing
- Output gain headroom: default -3dB, max limited to avoid clipping on this board

---

## Validation & Testing

- Repeated play/stop stress testing (50+ cycles)
- Ctrl-C abort scenario testing
- Register state monitoring
- Comparison with vendor reference driver

---

## Platform

- SoC: Rockchip RK3588
- Codec: Everest ES8388
- Board: Lubancat 5
- Kernel: Linux 6.1 (Android 15 BSP base)

---

## Scope Note

This project focuses on codec driver development and tuning.
Machine driver integration is handled by the platform BSP
and is outside the main scope of this repository.

---

## Repository Structure



# Phase3: DAPM Routing & Always-On Power State Tuning

## Goal

Construct a complete DAPM playback signal path and establish the
"always-on" power architecture that keeps analog blocks powered
between play/stop transitions to eliminate pop noise.

## Key Design Decision: All DAPM Widgets Use SND_SOC_NOPM

In a typical ASoC codec driver, DAPM widgets directly control hardware
power bits — for example, a `SND_SOC_DAPM_DAC` widget would reference
R04[7] to power on the left DAC when a stream starts.

This driver takes a different approach: **all DAPM widgets are
SND_SOC_NOPM** (no power management). The DAPM graph exists purely
for routing topology — it tells the framework which signal paths are
active, but never writes any power registers.

### Why?

The upstream `es8328.c` driver lets DAPM toggle individual power bits
in R02 and R04 on play/stop. This causes analog transients (pops)
because:

1. R04[5:4] (output amp) powering on/off creates DC offset changes
2. R02 (dacVref) toggling disturbs the internal reference voltage
3. DAPM settle delays (msleep) are needed, adding playback latency

By moving all power control into `set_bias_level` and keeping
R02/R04 always-on after boot, we eliminate both the pop noise and
the DAPM settle delays.

### Trade-off

| | Upstream es8328.c | This driver (always-on) |
|---|---|---|
| Idle current | Lower (~0.5mA) | Higher (+4-5mA) |
| Pop noise | Requires DAPM delays | None |
| Playback latency | ~200ms+ (settle) | Near-instant |
| Complexity | DAPM manages power | Simpler: mute bit only |

For battery-powered devices the upstream approach is better.
For development boards and AC-powered products, the always-on
approach gives superior audio quality with negligible power cost.

## DAPM Signal Path

```
I2S RX ──→ [DACL] ──→ [Left Mixer] ──→ [Left Out 1] ──→ LOUT1
Data        (NOPM)     (R27[7] sw)      (NOPM)

I2S RX ──→ [DACR] ──→ [Right Mixer] ──→ [Right Out 1] ──→ ROUT1
Data        (NOPM)     (R2A[7] sw)      (NOPM)
```

### Supply Chain (R02 dependencies)

```
[DAC STM] ──┐
[DAC Vref] ─┼──→ [DAC DIG] ──→ [DACL] / [DACR]
[DAC DLL] ──┘
```

These supply widgets model the hardware truth that the DAC digital
block (R02[6]) requires the state machine, Vref, and DLL to be
active. In this driver they are all NOPM — `set_bias_level` writes
R02=0x00 (all on) in a single operation during OFF→STANDBY.

## Mixer Switch Design

The mixer switches are the **only DAPM widgets that touch real
hardware registers**:

```c
SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL17, 7, 1, 0)
SOC_DAPM_SINGLE("Playback Switch", ES8388_DACCONTROL20, 7, 1, 0)
```

- R27[7] LD2LO: Left DAC → Left Output Mixer
- R2A[7] RD2RO: Right DAC → Right Output Mixer

These are set to 1 (enabled) via `reg_defaults` at probe time, so
DAPM discovers the path as active on boot. Userspace can toggle them
via tinymix for debug or routing changes.

## Power State Machine

### Register Ownership

Each register has exactly one owner to prevent conflicting writes:

| Owner | Registers | When |
|---|---|---|
| set_bias_level | R00 (VMID), R01 (protection), R02 (digital), R04 (DAC+amp) | Boot/suspend transitions |
| reg_defaults | R19, R1C, R2D, R27/R2A, R2E/R2F | Probe (regcache_sync) |
| mute_stream | R19[2] (mute bit only) | Play/stop |
| kcontrols | R1A/R1B (DAC vol), R2E/R2F (output PGA) | Userspace (tinymix) |
| hw_params | R17 (word length), R18 (MCLK ratio) | Stream open |

### Transition Detail

**Boot (OFF → STANDBY):**
1. R00: VMID=5k + ENREF (fast charge)
2. msleep(100) — capacitor charge time
3. R04: 0x30 (LDAC+RDAC on, LOUT1+ROUT1 on)
4. msleep(50) — output amp settle
5. R00: VMID=50k (switch to normal impedance)

**Play (STANDBY → PREPARE → ON):**
1. PREPARE: R02=0x00 (enable dacVref/DLL/STM/DIG)
2. ON: no-op (analog already powered)
3. mute_stream(0): R19[2]=0 (unmute)

**Stop (ON → PREPARE → STANDBY):**
1. mute_stream(1): R19[2]=1 (mute)
2. PREPARE → STANDBY: no power changes

**Suspend (STANDBY → OFF):**
1. R02=0xFF (digital off)
2. R04=0xC0 (DAC+output amp off)
3. R00: VMID=off, ENREF=off

## Validation

- `cat /sys/kernel/debug/asoc/components` — verify codec registered
- `cat /sys/kernel/debug/asoc/*/dapm_widgets` — check widget power state
- Register dumps via regmap debugfs before/after play/stop
- dmesg bias level transition logs

## What Changed from Phase2

| Area | Phase2 | Phase3 |
|---|---|---|
| DAPM widgets | Not present | Full playback graph |
| Power strategy | bias_level only | bias_level + always-on design |
| Mixer routing | Not connected | R27/R2A via reg_defaults + DAPM switch |
| Output PGA | Not exposed | kcontrols for R2E/R2F |
| mute_stream | Not present | R19[2] toggle only |

## Issues Encountered

- Writing R02 in PREPARE when value unchanged still causes I2C
  traffic that disturbs dacVref — discovered via pop noise under
  rapid play/stop cycles. Fixed in phase4 by removing the explicit
  R02 write in PREPARE.

- VMID must stay at 50k in both STANDBY and PREPARE. Switching to
  500k in STANDBY caused audible bias transient because the output
  amp (R04) remains powered.

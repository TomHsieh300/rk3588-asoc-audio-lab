# Phase4: Pop-Noise Suppression & Production Cleanup

## Goal

Eliminate remaining pop/click artifacts found during phase3 stress
testing, optimize playback startup latency, and clean up the driver
to production-quality code.

## Anti-Pop Architecture (Final)

### Problem Statement

Pop noise in codec drivers comes from two sources:

1. **Analog transients**: Powering on/off DAC, output amplifier, or
   VMID reference causes DC offset shifts at the output pin.
2. **dacVref disturbance**: Even redundant I2C writes to R02 (when
   the value is unchanged) cause a brief internal glitch in the
   DAC voltage reference, producing intermittent pop.

Phase3 solved (1) by keeping analog blocks always-on. Phase4 solves
(2) by removing the unnecessary R02 write in PREPARE.

### Root Cause: R02 Write in PREPARE

Phase3's `set_bias_level(PREPARE)` contained:

```c
snd_soc_component_write(component, ES8388_CHIPPOWER, 0);
```

This seems harmless — R02 is already 0x00 from STANDBY. But
`snd_soc_component_write()` always sends the I2C transaction
regardless of cached value (unlike `update_bits` which deduplicates).
The I2C write disturbs the dacVref circuit, causing intermittent pop
under rapid play/stop cycles (observed ~1 in 10 cycles).

### Fix

Remove the R02 write from PREPARE entirely. R02 is already 0x00
after the OFF→STANDBY transition and stays that way until
BIAS_OFF (suspend). Use `update_bits` for R00 (VMID), which has
built-in cache deduplication — no I2C occurs when already at 50k.

```c
case SND_SOC_BIAS_PREPARE:
    /* R02 already 0x00 from STANDBY — do NOT write it here.
     * R00 uses update_bits (cache dedup) so it's safe. */
    ret = es8388_update_bits(component, ES8388_CONTROL1,
                             ES8388_CONTROL1_VMIDSEL_MASK |
                             ES8388_CONTROL1_ENREF,
                             ES8388_CONTROL1_VMIDSEL_50k |
                             ES8388_CONTROL1_ENREF);
    break;
```

### Key Insight: write() vs update_bits()

| Function | Cache check | I2C on same value |
|---|---|---|
| `snd_soc_component_write()` | No | Always sends I2C |
| `snd_soc_component_update_bits()` | Yes | Skips if unchanged |
| `es8388_write()` (our wrapper) | No | Always sends I2C |
| `es8388_update_bits()` (our wrapper) | Yes | Skips if unchanged |

Rule: Use `update_bits` for registers that may already be at the
target value. Use `write` only when you need a guaranteed I2C
transaction (e.g., first-time setup in OFF→STANDBY).

## Soft Ramp (R19 Configuration)

```c
{ ES8388_DACCONTROL3, 0x26 }  /* mute=1, soft_ramp=1, 4LRCK/step */
```

R19 bit layout:
- [7:6] Ramp rate: 00 = 4 LRCK/step (fastest)
- [5]   Soft ramp enable: 1
- [2]   DAC mute: 1 (muted at boot)

The soft ramp makes mute/unmute transitions gradual rather than
instant, further reducing click artifacts. At 48kHz with 4 LRCK/step,
a full ramp from -96dB to 0dB takes ~2.5ms — inaudible to users but
enough to avoid the DAC output snapping between levels.

## Output Volume Safety

Phase4 changed output PGA defaults from -3dB to POR default (-45dB):

```c
/* Phase3: */
{ ES8388_LOUT1VOL, 0x1C },    /* -3dB — risky on unknown loads */

/* Phase4: */
{ ES8388_LOUT1VOL, 0x00 },    /* POR default (-45dB). Let userspace decide. */
```

Rationale: The codec driver should not assume the headphone/speaker
impedance. Setting high output gain in the driver can clip on
low-impedance loads. Let userspace (Android AudioHAL, PulseAudio,
amixer) set the appropriate level for the actual hardware.

## Register Ownership Refinements

Phase4 introduced explicit `#define` constants for all register
values used in `set_bias_level`, replacing magic numbers:

| Constant | Value | Meaning |
|---|---|---|
| `ES8388_DACPOWER_LOUT1_ROUT1` | 0x30 | DAC on + LOUT1/ROUT1 on |
| `ES8388_DACPOWER_ALL_OFF` | 0xC0 | DAC off, all outputs off |
| `ES8388_CHIPPOWER_ALL_ON` | 0x00 | All digital blocks enabled |
| `ES8388_CHIPPOWER_ALL_OFF` | 0xFF | All digital blocks disabled |
| `ES8388_VMID_CHARGE_MS` | 100 | 5k fast charge duration |
| `ES8388_DACPOWER_SETTLE_MS` | 50 | DAC + output amp settle |
| `ES8388_DACVREF_SETTLE_MS` | 50 | dacVref stabilization |

Phase3 also had the dacVref settle order wrong — R04 was written
before R02. Phase4 corrects this:

```
OFF → STANDBY (corrected):
  1. R00: VMID=5k, ENREF     (capacitor charge)
  2. msleep(100)
  3. R02: 0x00                (dacVref + DLL + STM on)
  4. msleep(50)               (dacVref settle — invisible because R04 still off)
  5. R04: 0x30                (output amp on — dacVref already stable)
  6. msleep(50)               (output amp settle)
  7. R00: VMID=50k            (switch to normal impedance)
```

The key: dacVref must settle **before** the output amplifier turns
on. If R04 powers on while dacVref is still charging, the transient
appears at LOUT1/ROUT1 as audible pop.

## Error Handling Improvements

Phase4 added `es8388_write()` and `es8388_update_bits()` wrappers
with:
- Return value checking at every call site
- `dev_err()` logging on failure
- Early return on error (fail-fast)

Phase3 ignored return values from `snd_soc_component_write()`, which
meant a failed I2C transaction would leave the codec in an
inconsistent state with no indication in dmesg.

## Stress Test Results

| Test | Phase3 | Phase4 |
|---|---|---|
| 50-cycle play/stop | ~5 pops | 0 pops |
| Ctrl-C abort | Occasional click | Clean |
| Rapid play/stop (<0.5s gap) | Intermittent pop | Clean |
| Cold boot to first play | 1 soft pop | Clean |

## What Changed from Phase3

| Area | Phase3 | Phase4 |
|---|---|---|
| R02 in PREPARE | Explicit write (pop source) | Removed (rely on STANDBY state) |
| dacVref timing | R04 before R02 | R02 before R04 (correct order) |
| Error handling | Ignored return values | Checked + dev_err at every site |
| Magic numbers | 0x30, 0xC0, 0xFF | Named constants |
| Output volume default | -3dB (risky) | -45dB POR (safe) |
| VROI | Not set | R2D=0x10 (40k to ground when off) |
| Register wrappers | Direct snd_soc_component calls | es8388_write/update_bits wrappers |

## Remaining Known Limitations

1. **DAC-only**: No ADC/capture path implemented
2. **LOUT2/ROUT2**: Widget graph includes them but R04 only enables
   LOUT1/ROUT1 (TODO: change to 0x3C when LOUT2/ROUT2 hw connected)
3. **No runtime PM**: suspend/resume only (no autosuspend)
4. **No MCLK-free detection**: If MCLK stops unexpectedly, I2C may
   still succeed but audio output will be silent with no error

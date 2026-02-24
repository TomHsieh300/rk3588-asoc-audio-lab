# Comparison: tom-es8388.c vs upstream es8328.c

This document compares the design decisions in this practice driver
against the upstream Linux kernel `sound/soc/codecs/es8328.c` driver
(torvalds/linux master branch). The upstream driver supports both
ES8328 and ES8388 chips as they are register-compatible.

Reference: https://github.com/torvalds/linux/blob/master/sound/soc/codecs/es8328.c

---

## Summary Table

| Aspect | upstream es8328.c | tom-es8388.c (this repo) |
|---|---|---|
| DAPM power control | Real register bits on widgets | All widgets SND_SOC_NOPM |
| R04 (DAC+amp) power | DAPM toggles per-widget bits | set_bias_level always-on |
| R02 (digital power) | Written in PREPARE | Not written in PREPARE |
| VMID in STANDBY | 500k (low power) | 50k (avoids bias transient) |
| Pop noise | Depends on DAPM settle delays | Eliminated by design |
| Playback latency | ~200ms+ (DAPM power-up delays) | Near-instant (mute bit only) |
| Idle current | Lower (~0.5mA analog off) | Higher (+4-5mA analog on) |
| Scope | Full DAC + ADC + Mic Bias | DAC-only (playback) |
| Bus transport | Separate I2C + SPI modules | Single-file I2C only |
| Mute mechanism | Not implemented (no mute_stream) | R19[2] via mute_stream callback |
| Error handling | Ignores write() return values | Checked + dev_err at every site |
| File structure | es8328.c + es8328.h + es8328-i2c.c + es8328-spi.c | Single tom-es8388.c |

---

## Detailed Analysis

### 1. DAPM Widget Power Strategy

**Upstream:**

Widgets reference real hardware register bits. DAPM framework
automatically powers blocks on/off when streams start and stop:

```c
/* upstream: DAPM controls R04 bits directly */
SND_SOC_DAPM_DAC("Left DAC", "Left Playback",
                  ES8328_DACPOWER, ES8328_DACPOWER_LDAC_OFF, 1),

SND_SOC_DAPM_PGA("Left Out 1", ES8328_DACPOWER,
                  ES8328_DACPOWER_LOUT1_ON, 0, NULL, 0),

SND_SOC_DAPM_MIXER("Left Mixer", ES8328_DACCONTROL17, 7, 0, ...),
```

**This driver:**

All widgets use SND_SOC_NOPM. Power is managed centrally by
set_bias_level. DAPM graph exists for routing topology only:

```c
/* tom: DAPM is routing-only, no register I/O */
SND_SOC_DAPM_DAC("DACL", "Playback", SND_SOC_NOPM, 0, 0),

SND_SOC_DAPM_PGA("Left Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0, ...),
```

**Why this matters:**

When DAPM controls R04 bit-by-bit, each widget power event causes
an I2C write to the same register (R04) at different times. The
output amplifier sees DC offset changes as the DAC and amp power
on/off asynchronously. This is the primary source of pop noise.

By moving all R04 control to set_bias_level (once at boot, once at
suspend), the entire analog chain powers up atomically in the correct
order with explicit settle delays.

### 2. VMID Impedance in STANDBY

**Upstream:**

```c
/* upstream: STANDBY uses 500k for lowest idle current */
case SND_SOC_BIAS_STANDBY:
    snd_soc_component_update_bits(component, ES8328_CONTROL1,
            ES8328_CONTROL1_VMIDSEL_MASK | ES8328_CONTROL1_ENREF,
            ES8328_CONTROL1_VMIDSEL_500k | ES8328_CONTROL1_ENREF);
```

**This driver:**

```c
/* tom: STANDBY uses 50k — same as PREPARE */
case SND_SOC_BIAS_STANDBY:
    es8388_update_bits(component, ES8388_CONTROL1,
            ES8388_CONTROL1_VMIDSEL_MASK | ES8388_CONTROL1_ENREF,
            ES8388_CONTROL1_VMIDSEL_50k | ES8388_CONTROL1_ENREF);
```

**Why this matters:**

In the upstream driver, DAPM powers off the output amp (R04) during
STANDBY, so VMID impedance changes are invisible at the output pin.

In this driver, the output amp stays powered. If VMID switches
between 500k (STANDBY) and 50k (PREPARE), the voltage divider ratio
changes, creating a bias transient visible at LOUT1/ROUT1 as an
audible click.

Trade-off: ~1mA higher idle current for glitch-free operation.

### 3. R02 (Chip Power) Write in PREPARE

**Upstream:**

```c
/* upstream: always writes R02 in PREPARE */
case SND_SOC_BIAS_PREPARE:
    snd_soc_component_write(component, ES8328_CHIPPOWER, 0);
    ...
```

**This driver (phase4):**

```c
/* tom: R02 is NOT written in PREPARE — already 0x00 from STANDBY */
case SND_SOC_BIAS_PREPARE:
    /* R00 uses update_bits (cache dedup) — safe, no I2C if unchanged */
    ret = es8388_update_bits(component, ES8388_CONTROL1, ...);
    break;
```

**Why this matters:**

`snd_soc_component_write()` always sends an I2C transaction even
when the register value is unchanged. Writing R02=0x00 when it's
already 0x00 disturbs the dacVref circuit, causing intermittent pop
noise (~1 in 10 rapid play/stop cycles).

This was the root cause of phase3's remaining pop noise, discovered
through stress testing. The fix is to simply not write R02 in
PREPARE — it's already in the correct state.

### 4. Mute Control

**Upstream:**

No `mute_stream` callback. The upstream driver relies entirely on
DAPM widget power toggling to control audio output. When a stream
stops, DAPM powers off the DAC widget (R04[7:6]) which silences
output. This means unmute/mute happens via analog power cycling.

**This driver:**

Explicit `mute_stream` callback that toggles only R19[2]:

```c
static int tom_es8388_mute(struct snd_soc_dai *dai, int mute, int dir)
{
    /* Only toggle DAC mute bit. Do NOT touch R1A/R1B (PCM Volume). */
    return es8388_update_bits(c, ES8388_DACCONTROL3,
                              ES8388_DACCONTROL3_DACMUTE,
                              mute ? ES8388_DACCONTROL3_DACMUTE : 0);
}
```

Combined with soft ramp (R19[5]=1, 4 LRCK/step), the mute/unmute
transition is ~2.5ms at 48kHz — inaudible but smooth.

### 5. Code Organization

**Upstream:**

```
es8328.c       — Core codec driver (DAPM, bias, DAI ops)
es8328.h       — Register definitions + shared declarations
es8328-i2c.c   — I2C transport probe
es8328-spi.c   — SPI transport probe
```

The split allows the same core driver to work over both I2C and SPI.
This is standard upstream practice for codecs supporting multiple
bus interfaces.

**This driver:**

```
tom-es8388.c   — Everything in one file
```

Single-file approach is appropriate for a practice/portfolio project
targeting a specific board (LubanCat5, I2C only). Splitting into
header + transport modules would be needed if submitting upstream
or if the codec needed SPI support.

### 6. Error Handling

**Upstream:**

Return values from `snd_soc_component_write()` and
`snd_soc_component_update_bits()` are not checked in set_bias_level
or mute operations. If an I2C transaction fails, the codec enters
an inconsistent state with no dmesg indication.

**This driver:**

Every register write is wrapped in `es8388_write()` /
`es8388_update_bits()` which log failures via `dev_err()` and
propagate error codes. The bias_level function uses early return
on error:

```c
ret = es8388_update_bits(component, ...);
if (ret)
    return ret;
```

### 7. Features Not in This Driver

The upstream driver includes several features not implemented here:

| Feature | upstream | tom-es8388 |
|---|---|---|
| ADC / capture path | Full LINPUT1/2, RINPUT1/2, PGA | Not implemented |
| Mic bias supply | SND_SOC_DAPM_SUPPLY for mic bias | Not implemented |
| Headphone detect | Via machine driver (imx-es8328.c) | Out of scope |
| SPI bus support | es8328-spi.c | Not needed (I2C only) |
| Left/Right DAI split | Separate "Left Playback" / "Right Playback" streams | Combined "Playback" stream |
| Deemphasis control | DACCONTROL6 deemph bits | Not implemented |

These are not limitations — they reflect the intentional scope of a
DAC-only codec bring-up exercise.

---

## When to Choose Which Approach

| Scenario | Recommended approach |
|---|---|
| Battery-powered device (phone, tablet) | Upstream (DAPM power control, 500k VMID) |
| AC-powered dev board / embedded system | Always-on (simpler, pop-free, instant) |
| Production codec driver for upstream | Upstream structure (split files, full ADC+DAC) |
| BSP bring-up / tuning | Always-on for initial validation, then optimize |
| Pop-sensitive application (hi-fi, medical) | Always-on + mute_stream + soft ramp |

---

## Lessons Learned

1. **DAPM is not always the right tool for power management.**
   DAPM excels at fine-grained power optimization for battery life,
   but for codecs with pop-sensitive analog paths, centralizing power
   control in set_bias_level gives better audio quality.

2. **`write()` vs `update_bits()` matters more than you'd expect.**
   A seemingly innocent write of the same value can disturb analog
   circuits through I2C bus activity. Always use `update_bits` for
   registers that may already be at the target value.

3. **VMID impedance affects more than idle current.**
   In an always-on architecture, VMID switches between states create
   audible artifacts at the output. Keeping VMID constant between
   STANDBY and PREPARE eliminates this class of pop noise.

4. **Upstream drivers are not always bug-free.**
   The upstream es8328.c has known issues (no mute_stream, no return
   value checking, mixer routing patches still being submitted as of
   2025). Reading upstream code is essential, but blind copying is not
   sufficient for production-quality audio.

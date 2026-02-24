# ES8388 Driver Architecture — Signal Path & Power Ownership

## Hardware Block Diagram (ES8388 DAC Side)

```
                         ES8388 Internal Architecture (DAC Path)
 ┌──────────────────────────────────────────────────────────────────────────────────┐
 │                                                                                  │
 │  ┌──────────┐     ┌─────────────┐     ┌────────────┐                             │
 │  │ I2S RX   │     │  DAC Core   │     │  Output    │     ┌───────┐               │
 │  │ (Serial  │────▶│  L-ch / R-ch│────▶│  Mixer     │────▶│ LOUT1 │───────▶ HP_L  │
 │  │  Data)   │     │  (R04[7:6]) │     │  (R27/R2A) │     │ (R04) │               │
 │  └──────────┘     └──────┬──────┘     └─────┬──────┘     └───────┘               │
 │                          │                  │             ┌───────┐              │
 │                          │                  └────────────▶│ ROUT1 │──────▶ HP_R  │
 │                          │                                │ (R04) │              │
 │                          │                                └───────┘              │
 │                          │                                                       │
 │              ┌───────────┴───────────┐                                           │
 │              │   Digital Power (R02) │                                           │
 │              │  dacVref │ DLL │ STM  │                                           │
 │              └───────────┬───────────┘                                           │
 │                          │                                                       │
 │              ┌───────────┴───────────┐                                           │
 │              │ Analog Reference (R00)│                                           │
 │              │   VMID    │   VREF    │                                           │
 │              └───────────────────────┘                                           │
 │                                                                                  │
 └──────────────────────────────────────────────────────────────────────────────────┘
```

## DAPM Widget Graph (Kernel View)

```
 "Playback" stream
      │
      ▼
 ┌──────────┐     ┌──────────┐
 │   DACL   │     │   DACR   │      SND_SOC_DAPM_DAC (NOPM)
 │ (NOPM)   │     │ (NOPM)   │
 └────┬─────┘     └────┬─────┘
      │                │
      ▼                ▼
 ┌──────────┐     ┌──────────┐
 │   Left   │     │  Right   │      SND_SOC_DAPM_MIXER (NOPM body)
 │  Mixer   │     │  Mixer   │      Switch control: R27[7] / R2A[7]
 │ "PB Sw"  │     │ "PB Sw"  │
 └────┬─────┘     └────┬─────┘
      │                │
      ▼                ▼
 ┌──────────┐     ┌──────────┐
 │ Left     │     │ Right    │      SND_SOC_DAPM_PGA (NOPM)
 │ Out 1    │     │ Out 1    │
 └────┬─────┘     └────┬─────┘
      │                │
      ▼                ▼
 ┌──────────┐     ┌──────────┐
 │  LOUT1   │     │  ROUT1   │      SND_SOC_DAPM_OUTPUT
 └──────────┘     └──────────┘

 Supply chain (all NOPM, documents hardware dependencies):

 [DAC STM] ──┐
 [DAC Vref]──┼──▶ [DAC DIG] ──▶ DACL / DACR
 [DAC DLL] ──┘
```

## Register Ownership Map

```
 ┌─────────────────────────────────────────────────────────────────┐
 │                    Register Ownership (Phase4)                  │
 ├─────────────────────────────────────────────────────────────────┤
 │                                                                 │
 │  set_bias_level (power lifecycle)                               │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R00  VMID + VREF reference                                │  │
 │  │ R01  Overcurrent + thermal protection                     │  │
 │  │ R02  Digital power (dacVref / DLL / STM / DIG)            │  │
 │  │ R04  DAC engine + output amp (LOUT1/ROUT1)                │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 │  mute_stream (play/stop — the ONLY thing that toggles)          │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R19[2]  DAC mute bit                                      │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 │  hw_params (stream configuration)                               │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R17  DAC word length (16/18/20/24/32-bit)                 │  │
 │  │ R18  MCLK/LRCK ratio (256x, 384x, etc.)                  │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 │  reg_defaults (probe-time setup via regcache_sync)              │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R17  I2S format default                                   │  │
 │  │ R18  256x ratio default                                   │  │
 │  │ R19  Mute=1, soft_ramp=1, 4LRCK/step                      │  │
 │  │ R1C  Clickfree enable                                     │  │
 │  │ R27  Left mixer: DAC→output ON, bypass atten=0dB          │  │
 │  │ R2A  Right mixer: DAC→output ON, bypass atten=0dB         │  │
 │  │ R2D  VROI: 40k to ground when output off                  │  │
 │  │ R2E  LOUT1 volume: -45dB POR default                      │  │
 │  │ R2F  ROUT1 volume: -45dB POR default                      │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 │  kcontrols (userspace: tinymix / amixer / AudioHAL)             │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R1A  Left DAC digital volume  (0dB to -96dB, 0.5dB step)  │  │
 │  │ R1B  Right DAC digital volume                             │  │
 │  │ R2E  LOUT1 output PGA (-45dB to +4.5dB, 1.5dB step)       │  │
 │  │ R2F  ROUT1 output PGA                                     │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 │  DAPM switches (tinymix "Playback Switch")                      │
 │  ┌───────────────────────────────────────────────────────────┐  │
 │  │ R27[7]  LD2LO: Left DAC → Left Mixer enable               │  │
 │  │ R2A[7]  RD2RO: Right DAC → Right Mixer enable             │  │
 │  └───────────────────────────────────────────────────────────┘  │
 │                                                                 │
 └─────────────────────────────────────────────────────────────────┘
```

## Power State Machine

```
                    Power State Transitions
 ═══════════════════════════════════════════════════════

 [BIAS_OFF] ──────────────────────────────────▶ [BIAS_STANDBY]
   (suspend)         Boot / Resume                (idle)
                                                    │
   R00: VMID=5k + ENREF ─── msleep(100) ────┐       │
   R02: 0x00 (all digital on) ─ msleep(50) ─┤       │
   R04: 0x30 (DAC+amp on) ──── msleep(50) ──┤       │
   R00: VMID=50k ───────────────────────────┘       │
                                                    │
                                                    ▼
 [BIAS_STANDBY] ◀═══════════════════════▶ [BIAS_PREPARE]
    (idle)          Play / Stop              (streaming)
                                                │
   VMID stays 50k (no change!)                  │
   R02 stays 0x00 (no write!)                   ▼
   R04 stays 0x30 (no change!)            [BIAS_ON]
                                           (active)
   ┌─────────────────────────────────┐
   │ mute_stream is the ONLY toggle: │
   │   Play: R19[2] = 0 (unmute)     │
   │   Stop: R19[2] = 1 (mute)       │
   └─────────────────────────────────┘

 [BIAS_STANDBY] ──────────────────────────────▶ [BIAS_OFF]
   (idle)              Suspend                   (power off)

   R02: 0xFF (digital off)
   R04: 0xC0 (DAC + amp off)
   R00: VMID=off, ENREF=off
```

## Volume Signal Chain

```
 I2S Data ──▶ [DAC Digital Volume] ──▶ [DAC Core] ──▶ [Mixer] ──▶ [Output PGA] ──▶ LOUT1/ROUT1
              R1A/R1B                                              R2E/R2F
              0dB to -96dB                                         -45dB to +4.5dB
              (0.5dB step)                                         (1.5dB step)

                     ▲                                                  ▲
                     │                                                  │
                "PCM Volume"                               "Output 1 Playback Volume"
                 (tinymix)                                          (tinymix)

             ┌─────────────────┐                               ┌─────────────────┐
             │ Safe default:   │                               │ Safe default:   │
             │ 0xC0 = -96dB    │                               │ 0x00 = -45dB    │
             │ (POR, silent)   │                               │ (POR, minimal)  │
             └─────────────────┘                               └─────────────────┘
```

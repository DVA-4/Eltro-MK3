# Eltro MK3 — Technical Notes

A digital emulation of the **Eltro Tempophon** (the rotating-head tape
pitch/time changer famously used to create HAL 9000's voice in *2001*). Live
audio in → ring buffer ("tape") with four Hann-windowed virtual read heads →
audio out, decoupled into independent time and pitch axes.

The machine is modelled as **two independent axes** — progression (time)
driven by the tape transport, and pitch driven by the relative (tape − drum)
velocity — so time and pitch can be changed independently, the way the real
machine does. Built as an Arduino sketch for the **AMYboard** (ESP32-S3
Eurorack module) using the **AMY** synth library.

---

## Hardware

| Part | Role | I2C addr | Bus |
|------|------|----------|-----|
| AMYboard (ESP32-S3) | host + audio I/O | — | — |
| M5Stack 8Angle | 8 pots + toggle + 9 RGB LEDs | `0x43` | front Grove |
| SH1106 128×64 OLED | display | `0x3C` | front Grove |
| ADS1115 | 2× CV input (ADC) | `0x48` | onboard AMYboard — not an external peripheral |

- **Front-panel Grove I2C: SDA = GPIO 17, SCL = GPIO 18** (not 21/22 — that's
  bare-DevKit only; accessories go on the FRONT panel, not the back "host" port).
- **MIDI IN (TRS): GPIO 21** (`AMYBOARD_MIDI_IN`), independent of I2C.
- Audio: 44.1 kHz, block size 256, stereo (`AMY_NCHANS = 2`).
- Audio out = **PCM5101 DAC** (analog line out). Audio in / SPDIF = **PCM9211**
  — these are separate chips; the analog-out path does not go through the PCM9211.

### Eurorack panel
- 26HP panel, 3U (131.8 mm × 128.5 mm). Keep controls within the central
  ~110 mm of height to clear the rails/screw slots.
- The 8Angle is 128 mm long — too long to mount vertically; mounted
  horizontally (knobs in a row across the panel width), remove PCB from case.

---

## Audio architecture

**Uses AMY's external-buffer path, not a render hook.** This is the key
lesson for anyone building a real-time audio-in effect on AMYboard:

```c
loop() (core 1, audio only):
    amy_get_input_buffer(buf);        // pull ADC input block (int16 stereo)
    eltroProcessBlock(buf);           // downmix -> effect -> write mono to L+R
    amy_set_external_input_buffer(buf);
    amy_update();                     // render + I2S write; blocks ~1 block
```

- Two oscillators, `AUDIO_EXT0` (pan hard-left) and `AUDIO_EXT1` (pan
  hard-right), set up under one synth, both `amp_coefs[COEF_CONST] = 1.0`.
- Buffer is int16, interleaved stereo (`buf[2n]=L, buf[2n+1]=R`), length
  `AMY_BLOCK_SIZE * AMY_NCHANS`. No fixed-point conversion needed at the
  boundary — AMY's `L2S()` handles it internally.
- Mono effect: downmix `(L+R)/2`, run the Eltro engine, write the result to
  both L and R.
- `amy_update()` belongs in `loop()` here — it blocks on AMY's background fill
  task and paces the loop. This is the opposite of the render-hook model.

**Why not the render hook:** `amy_external_render_hook` + an `AUDIO_IN0`
oscillator produced an amplitude-independent square wave (MSB-only output)
that survived every fix attempted. The external-buffer path (the official
`AMY_custom_dsp` example) is the correct tool for audio-in passthrough/processing.

---

## The Eltro DSP model — two independent axes

The buffer is a **pre-written tape**: a single write head records the live
input at a constant +1.0 (unit density), independent of everything — tape
speed does not touch the write. All the character comes from how the tape is
*read*.

**Two axes, not one velocity:**

1. **Progression (time axis) = tape.** A single shared grab playhead
   (`s_grab`) walks through the buffered tape at the tape rate
   (`s_grab += tapeSpeed` per sample). Depends on tape only — never on drum.
2. **Pitch axis = (tape − drum).** Within each segment, each of the 4 read
   heads scans its material at the relative velocity
   (`scanRate = tape − drum`). This re-pitches the grabbed material without
   changing how fast progression moves through it.

**Reconciliation (the grain mechanism).** A head reads at
`s_segOrigin[h] + s_segScan[h]`. When a head's scan crosses one segment length
(in either direction), it grabs a fresh segment at the current `s_grab` and
resets its scan. Segments repeat or drop to reconcile the two rates; a Hann
crossfade at the segment edges hides the handoff — essentially Wendy Carlos's
description of "throwing out many itty-bitty bits of the recording,
cross-fading the remaining ones together." The four heads stagger by scan
phase within the segment, so at any instant one head is mid-segment while
another hands off, giving continuous output.

The read heads are **phase-locked** to one master scan phase
(`s_masterScan`, advancing at `tape − drum`), with each head's position
derived as `master + (arcLen/NUM_HEADS)*h`. This keeps the four-head stagger
correct by construction at any buffer/segment length — an earlier
implementation with independent per-head accumulators would drift and produce
a stutter when the buffer or segment length changed at runtime.

**Physics this reproduces** (confirmed on hardware):

| Condition | Result |
|-----------|--------|
| tape == drum | silence / freeze — scan frozen |
| tape − drum == 1 | recorded pitch (unity) — scans at write density |
| drum > tape | reverse scan within the grain |
| tape == 0 | freeze on a spot — grab playhead frozen |
| tape fixed, drum varied | progression HOLDS, pitch MOVES (the decoupling) |

Output pitch = input × (tape − drum), independent of progression rate.
Progression through the tape = tape × sample rate, independent of drum.

`tape` is progression/time (rate through the recording). `drum` enters only
via (tape − drum) = pitch — it is a relative velocity, not a "pitch knob" on
its own.

**Why two axes:** a real-time single-pass effect can't time-stretch a live
stream — the decoupling operates over buffered tape (up to ~20 s of recorded
history) rather than future input. The design goal was the closest
approximation of what the hardware Tempophon actually does mechanically,
rather than a textbook granular time-stretch built backward from "preserve
pitch, change duration."

### Segment length and overlap are the tone controls
Segment length is the grain size — how much tape each head scans before
grabbing a fresh segment — absolute in samples, linear, independent of buffer
length: 64 samples (~1.5 ms, buzzy) to 32768 samples (~743 ms, smooth),
default 4096 (~93 ms). Overlap (0.10–0.95, default 0.25) controls crossfade
depth; the grain handoff clicks without enough crossfade, and seams become
audible below ~0.10.

### Speed-knob taper
The tape and drum knobs map through an exponential taper (`out = sign(x) ·
|x|^γ · range`) so the musically dense −1…+1 region around the centre detent
gets more knob travel. Default γ = 2.0. Tunable live over serial and
persisted to flash.

---

## Front-panel LEDs
Computed from `readthru = tape − drum` and `shift = readthru − 1`, using the
effective pitch axis (knob + CV + MIDI), so the ring reflects what's actually
happening rather than just knob position: dim red at silence, green at unity,
blue when faster/up, amber when slower/down. Bypass shows all-red, Page B
shows all-green. A turned knob flashes white briefly over whatever colour is
showing.

---

## Dual-core split (required for continuous audio)
The OLED full-frame flush is ~100 ms of blocking I2C. On a single core this
starves the audio loop. Fix:

- **Core 1 (`loop()`): audio only.** Touches no I2C.
- **Core 0 (`uiTask`):** all I2C — pots, LEDs, OLED flush, MIDI parse. Sole
  owner of the Wire bus, so there's no cross-core I2C race and no mutex needed.
- Shared parameters are `volatile` and word-sized → atomic on ESP32.
- Healthy throughput indicator: ~172 blocks/s (44100/256). Well below that
  means something is blocking the audio loop.

## Setup ordering: `amy_start()` before any Wire/I2C

`amy_start()` configures the PCM9211/PCM5101 codecs over I2C and brings up the
I2S clock/framing from the pristine power-on bus. If any I2C runs first
(`Wire.begin()`, pot reads, LED writes, OLED init), the I2S TX framing latches
wrong on cold power-up — loud, distorted, level-independent analog output,
even though the digital samples are clean. The latch survives warm reset and
reflash (it's codec/I2S clock state, not CPU state) and clears only on a
power cycle or a TX-only I2S init.

**Order:**
1. Before `amy_start()`: USB/CDC + Serial, Serial1 MIDI setup, non-I2C state.
   No Wire.
2. `amy_start(cfg)`.
3. After `amy_start()`: one `Wire.begin()/setClock()` (sole bus bring-up),
   then pots/LEDs, then display init, then oscillator events, then the UI task.

---

## MIDI

**RX:** byte-at-a-time state-machine parser with running-status support
(replacing a naive fixed 3-byte reader, which mis-frames on running status).
Tracks running status, ignores real-time bytes, dispatches only Control
Change. Runs in `uiTask` (core 0), bounded per pass so a CC flood can't
starve the UI.

Fixed CC map (Page A knobs), all 7-bit, RX channel selectable:

| CC | Param | Mapping |
|----|-------|---------|
| 102 | tape | bipolar taper (same law as the knob) |
| 103 | drum | bipolar taper |
| 104 | overlap | 0.10–0.95 |
| 105 | wow | 0.0–0.05 |
| 106 | input gain | 0.0–4.0 |
| 107 | output gain | 0.0–1.5 |
| 108 | decay | 0.0–0.9999 |
| 109 | buffer length | log 512–884736 |
| 65 | page toggle | ≥64 → Page B |

CCs are chosen from the spec's undefined range (102–119) to avoid colliding
with standard controllers.

**Catch-style takeover (per knob).** An incoming CC sets the parameter and
marks that knob "MIDI-owned"; the physical pot is then ignored for that
parameter until it moves past a raw-ADC threshold. Ownership is per-knob, so
you can automate one parameter over MIDI while tweaking another by hand. The
OLED shows a small marker next to any MIDI-owned Page-A knob.

**TX:** knob-turn → CC out (same CC map as RX), physical-move-only, and
guarded so it never echoes an incoming CC back out (skips any knob currently
MIDI-owned). Deadband is 7-bit re-quantization — emits only when the outgoing
0–127 value changes.

**Electrical note on MIDI-out:** the AMYboard MIDI-out is not a simple
single-pin UART TX — it brings two GPIOs to the two TRS-out contacts. A
DIN-4/5 opto input needs both legs actively defined (it's a current loop), so
MIDI TX is driven **differentially**: UART1 TX normal on one GPIO, its
hardware-inverted mirror on a second GPIO via the ESP32-S3 GPIO matrix. A
single-ended drive (TX pin + a floating or static-LOW return) produced
silence or garbled data into test interfaces. This is a clever way the board
delivers a 5V-era MIDI signal cleanly from 3.3V logic; it is not a
single-ended-plus-ground design.

---

## CV inputs
Two CV inputs are read from an ADS1115 (I2C `0x48`, single-ended A0/A1) in
`uiTask`, alternating (ping-pong) single-shot conversion at ~50 Hz, lightly
smoothed, mapped 1 unit/volt and clamped to ±5.0. CV1 → tape, CV2 → drum,
both additive (`effectiveTape = tape + cv·scale + offset`), with per-channel
scale/offset available in the parameter pages. Unpatched inputs float at a
nonzero reading, so CV amount should be kept at 0 until a cable is patched.

---

## Toolchain

**Known-good pairing:** arduino-esp32 core 3.3.8+ together with AMY library
1.2.14+ (or `main`). The AMYboard board target itself only exists from core
3.3.x onward, so you cannot go below 3.3.8 and still select the AMYboard
board.

**A version-skew failure mode worth knowing about:** if the board package is
updated but the AMY library is left on an older release, synth-out (I2S TX
only) still works cleanly, but audio-in → out (full-duplex RX+TX) produces
loud, very distorted, level-independent garbage — even though the digital
samples in the buffer are provably clean. This looks like a code regression
but isn't; the fix is updating AMY to match the core, not rolling the core
back.

If you move to a newer core, update AMY in lockstep and re-validate the
audio-in path with a minimal passthrough sketch before assuming a full
instrument sketch is at fault.

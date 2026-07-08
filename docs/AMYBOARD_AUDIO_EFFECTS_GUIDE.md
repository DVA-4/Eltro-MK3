# Building Audio Effects on the AMYboard — Practical Guide

Patterns and hard-won lessons for **real-time audio in -> process -> out**
effects on the **AMYboard** (ESP32-S3 + AMY library), with an **8Angle** for
control and an **OLED** for display. Generalised from the Eltro MK II build.

---

## 1. The audio path: use the external-buffer API

For any "process live audio" effect, use AMY's **external-buffer** path. Do NOT
use a render hook for audio-in passthrough/processing.

```c
// setup(): enable audio in, define two AUDIO_EXT oscillators (stereo)
amy_config_t cfg = amy_default_config();
cfg.features.audio_in = 1;
amy_start(cfg);

amy_event e = amy_default_event();
e.reset_osc = RESET_AMY; amy_add_event(&e);

e = amy_default_event();
e.synth = 18; e.num_voices = 1; e.oscs_per_voice = 2; amy_add_event(&e);

// osc 0 = left, osc 1 = right
e = amy_default_event(); e.synth=18; e.osc=0; e.wave=AUDIO_EXT0;
e.pan_coefs[COEF_CONST]=0.0f; e.amp_coefs[COEF_CONST]=1.0f; e.velocity=1.0f;
amy_add_event(&e);
e = amy_default_event(); e.synth=18; e.osc=1; e.wave=AUDIO_EXT1;
e.pan_coefs[COEF_CONST]=1.0f; e.amp_coefs[COEF_CONST]=1.0f; e.velocity=1.0f;
amy_add_event(&e);
// then note-on both EXT oscs to activate them.
```

```c
// loop() = AUDIO ONLY:
amy_get_input_buffer(buf);          // int16 stereo interleaved, BLOCK_SIZE*NCHANS
processBlock(buf);                  // your DSP, in place
amy_set_external_input_buffer(buf);
amy_update();                       // blocks ~1 block; paces the loop
```

### Buffer format
- `output_sample_type` = **int16**, **interleaved stereo**: `buf[2n]=L`,
  `buf[2n+1]=R`. Length = `AMY_BLOCK_SIZE * AMY_NCHANS`.
- Work in plain int16 (or convert to float ±32768 and back). **Do NOT do AMY
  s8.23 fixed-point (F2S) at this boundary** — AMY handles that internally via
  `L2S()`. Manual F2S here is how you get silence or square-wave garbage.
- Mono effect on stereo I/O: downmix `(L+R)/2`, process once, write result to
  both L and R.

### Why not the render hook
`amy_external_render_hook` + an `AUDIO_IN0` oscillator is the "obvious" approach
and it is a trap for audio-in effects. It forces you into per-osc s8.23 buffers,
pan/amp coef gymnastics, and (empirically) produced an **amplitude-independent
square wave** that no amount of scaling fixed. The external-buffer path is the
official, working pattern (`AMY_custom_dsp` example). Reach for it first.

---

## 2. Dual-core is mandatory if you have an OLED

A full-frame SH1106/SSD1306 flush over 100 kHz I2C takes **~100 ms** (blocking).
If the audio loop and the display live on the same core, the flush starves audio
and you get periodic dropouts (~10-20 per second). The audio block rate at
44.1k/256 is **~172 blocks/s**; watch that number — if it falls well below 172,
something is blocking your audio loop.

**Split across the two cores:**
- **Core 1 (`loop()`):** audio pump only. Touches NO I2C.
- **Core 0 (`xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, NULL, 0)`):**
  all I2C — pots, LEDs, OLED, MIDI parse + MIDI send.

### I2C ownership rule (avoids the nastiest bug class)
Make **one task the sole owner of the Wire bus.** When the audio core touches no
I2C and the UI task owns it exclusively, there is **no cross-core I2C race and no
mutex needed.** Cross-core I2C contention on a shared bus is a brutal,
intermittent bug — design it out by ownership, not by locking.

### Init ordering: `amy_start()` before any Wire/I2C
Separate from the runtime ownership rule above: at startup, `amy_start()` must run
**before** the first `Wire.begin()` or any I2C traffic. `amy_start()` configures
the codecs over I2C and brings up I2S framing from the pristine power-on bus;
touching I2C first latches the I2S **TX** framing wrong on **cold boot** (loud,
distorted, level-independent analog out, but clean digital samples — and it
survives reset/reflash, clearing only on a power cycle). Do non-I2C setup +
`amy_start()` first, then bring up Wire and all peripherals. The official example
cold-boots clean precisely because it touches no I2C before `amy_start()`.

### Sharing state between cores
DSP params written by the UI task and read by the audio task: declare `volatile`
and keep them **word-sized** (float, int, bool). Those reads/writes are atomic on
ESP32 — no mutex. Don't share multi-word structs without protection.

---

## 3. Control surface (8Angle)

| Function | I2C (addr `0x43`) |
|----------|-------------------|
| Read pot ch (0-7) | write reg `ch*2`, read 2 bytes LE (12-bit, 0-4095) |
| Read toggle | write reg `0x20`, read 1 byte, bit 0 |
| Set RGB LED ch | write reg `0x30 + ch*4`, then R, G, B, brightness |

### Orientation: keep TWO separate concepts
- **Channel order** (which physical knob is logical N): flips if you mount the
  board rotated. Remap logical->hardware at BOTH the pot read AND the LED write
  so everything downstream (params, LEDs, labels) stays consistent.
- **Turn direction** (CW increases vs decreases): **electrical, independent of
  mounting.** Rotating the board does NOT change a pot's CW sense. Use a separate
  invert flag (`v = 4095 - v`). Do not tie direction to the mounting flag.

### Pot smoothing / deadband
Use a deadband (e.g. ignore changes < ~20 LSB) before treating a pot as "moved",
to avoid jitter and to drive "last touched knob" display logic. The same
"moved?" gate is what MIDI TX hangs off of (send a CC only on real movement).

---

## 4. Display (OLED)

- Lives on the UI core (it's the ~100 ms blocker — keep it off audio).
- 10 fps (`DISPLAY_MS = 100`) is plenty for knob feedback and cheap.
- Watch horizontal overflow: a 128px-wide SH1106 at a 6px font fits **~21
  chars/row**. Strings longer than that silently clip off the right edge — a
  value can be "in the code" but invisible. Budget pixels per row.
- Prefer gauges over redundant numbers when space is tight; show the precise/most
  musical value (e.g. pitch in cents) once, prominently.

---

## 5. Gain staging

AMYboard analog audio-in is **quiet**. Either bump the EXT oscillator amp
(`amp_coefs[COEF_CONST]`, the official example uses ~5.0) OR provide an input-gain
control in your effect (cleaner — keeps the osc at unity and avoids surprise
loudness). Provide an output gain too. Watch for clipping when feedback/decay
effects accumulate energy.

---

## 6. Debugging methodology (this saved the project)

- **Build markers.** Print `Ready [BUILD: rev-X]` on boot. ALWAYS confirm the
  marker on-device matches the file you flashed. Editor/flash desync wastes hours
  of "fixes that did nothing because they weren't running."
- **Measure, don't assume.** Print raw sample values; scope the actual analog
  out. An "envelope that matches" is not proof the waveform is intact. We chased
  several wrong theories that a single raw-sample dump or scope trace killed
  instantly. (The MIDI-out fix was pure scope work: pin → jack → DIN pins.)
- **Isolate with minimal sketches.** A bare "AMY plays one sine" sketch and a
  bare "native audio-in passthrough" sketch partitioned the problem (DAC path vs
  input path vs our code) far faster than reasoning about the full sketch. A tiny
  standalone "spam a CC out" sketch did the same for MIDI TX.
- **Amplitude sweep test.** If output looks wrong, drive the input at 0.8 / 0.1 /
  0.01 and scope it. **Same shape at all three = sign/format/MSB bug**
  (amplitude-independent). **Scales with input = gain/clipping.** These need
  opposite fixes; the sweep tells you which instantly.
- **Treat disproven theories as real signal.** Each "that didn't fix it" narrows
  the space — don't re-pursue a killed hypothesis without new evidence.

- **When mutating-toward-broken stalls, bisect from the GOOD side.** If poking at
  your broken sketch keeps hitting confounds, start from a *known-good minimal
  example* and inject your sketch's differences into it ONE at a time until it
  breaks. The injection that flips it is the cause, isolated from everything else.
  This is what finally cracked the cold-boot latch: AMY's bare example cold-booted
  clean, so we added Eltro's `setup()` deltas to it cumulatively (midi config →
  manual USB → pre-`amy_start` I2C) and the break tracked exactly to I2C-before-
  `amy_start()`. Mutating the other direction had buried it under OLED-freeze and
  version-skew noise for a whole prior session.

- **A clean digital dump with garbage analog points at the I2S/clock layer, not
  your DSP.** If `out16` (or your equivalent sample dump) is a perfect waveform
  while the analog out is mangled, stop auditing your code — the corruption is in
  I2S framing / codec clocking. RX-data bugs show up *in* the dump; TX-framing
  bugs don't. (Same logic for MIDI: garbled-arriving vs nothing-arriving point at
  different layers — see the programming guide's MIDI-out section.)

---

## 7. Quick reference

- Front Grove I2C: **SDA 17, SCL 18** (not 21/22).
- MIDI IN: **GPIO 21**, 31250 baud, RX only. Input is polarity-agnostic
  (bridge rectifier ahead of the opto — accepts TRS Type-A or B).
- MIDI OUT: **differential** — UART1 TX normal on **GPIO 14**, inverted mirror on
  **GPIO 15** via the GPIO matrix. NOT single-ended. See the programming guide.
- Audio: 44.1 kHz, block 256, stereo, `AMY_NCHANS = 2`.
- Output DAC: PCM5101 (analog). Input/SPDIF: PCM9211. Separate chips.
- Healthy audio loop: **~172 blocks/s**.

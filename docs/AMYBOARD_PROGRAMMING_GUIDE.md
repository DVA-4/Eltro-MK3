# Programming the AMYboard — General Guide

Foundational notes for building anything on the **AMYboard** (ESP32-S3 + AMY
synth library) in the Arduino environment, with I2C peripherals (8Angle, OLED,
ADCs, ...) and notes on real-time MIDI. Distilled from the Eltro MK II project so
future builds don't relearn it the hard way.

---

## 1. Board basics

- MCU: **ESP32-S3** (dual core, PSRAM). Arduino board target **AMYboard** sets
  `-DAMYBOARD_ARDUINO` and `cdc_on_boot=0`.
- **`AMYBOARD_ARDUINO` vs `AMYBOARD` are NOT the same define.** The Arduino target
  defines `AMYBOARD_ARDUINO` only. Library code gated on plain `defined(AMYBOARD)`
  (some MIDI/parser paths) is silently EXCLUDED on Arduino builds. If something in
  AMY "should be there" but isn't compiling in, check which of the two defines
  gates it.
- **USB CDC**: with `cdc_on_boot=0`, set up USB-CDC manually:
  ```c
  #if !ARDUINO_USB_CDC_ON_BOOT
  #include "USB.h"
  USBCDC USBSerial;
  #define Serial USBSerial
  #endif
  // in setup(): USB.begin(); Serial.begin(115200);
  ```
- **Do not enable AMY's USB-MIDI** (`cfg.midi = AMY_MIDI_IS_NONE`) when you run
  your own manual USB-CDC — the USB gadget descriptors conflict and can hang boot.

### Pin map that actually matters
| Signal | Pin | Notes |
|--------|-----|-------|
| Front Grove I2C SDA | **GPIO 17** | accessories go here, NOT 21/22 |
| Front Grove I2C SCL | **GPIO 18** | (21/22 is bare-DevKit only) |
| MIDI IN (TRS) | **GPIO 21** | `AMYBOARD_MIDI_IN`, independent of I2C |
| MIDI OUT contact A | **GPIO 14** | `AMYBOARD_MIDI_OUT_TYPE_A`, → tip → DIN pin 5 |
| MIDI OUT contact B | **GPIO 15** | `AMYBOARD_MIDI_OUT_TYPE_B`, → ring → DIN pin 4 |
| I2S (audio) | internal | handled by AMY; don't touch |

- MIDI OUT is **two GPIOs, one per TRS contact, each via a 22 Ω series resistor**
  — it is a differential/current-loop output, not a single-ended TX pin. See §5.
- Audio chips: **PCM5101** = analog line OUT (DAC, no I2C, config-free).
  **PCM9211** = analog/SPDIF IN + SPDIF out (I2C `0x40`). **ADS1015** = CV ADC
  (I2C). These are distinct — the analog-out path does not pass through the
  PCM9211.

---

## 2. AMY startup skeleton

```c
amy_config_t cfg = amy_default_config();
cfg.features.startup_bleep  = 0;
cfg.features.default_synths = 0;
cfg.features.chorus = 0; cfg.features.reverb = 0;   // disable what you don't use
cfg.features.audio_in = 1;                          // if you need input
cfg.midi = AMY_MIDI_IS_NONE;                        // if doing your own MIDI/USB
amy_start(cfg);
```

- Defaults: 44.1 kHz, block size 256, `AMY_NCHANS = 2`, `multithread = 1`.
- With `multithread = 1`, AMY runs a **background fill task**; `amy_update()`
  **blocks** on it (`ulTaskNotifyTake(portMAX_DELAY)`). That blocking is the
  intended pacing mechanism when you drive audio from `loop()`.
- Whether you call `amy_update()` in loop depends on your model:
  - **External-buffer / audio-in effects:** YES — it pulls your buffer and paces
    the loop (see the audio-effects guide).
  - **Pure synth with no per-loop buffer work:** AMY's background task already
    drives audio; an extra `amy_update()` may just block. Match the official
    example for your use case.

### ⚠️ Call `amy_start()` BEFORE any Wire/I2C (codec bring-up ordering)

`amy_start()` configures the PCM9211/PCM5101 over I2C and brings up I2S
clock/framing from the **pristine power-on bus**. If you touch I2C first
(`Wire.begin()`, peripheral reads, OLED init), the I2S **TX** framing can latch
wrong on **cold power-up**: loud, distorted, level-independent analog out with
*clean digital samples*. It survives reset/reflash (codec/I2S clock state, not
CPU state) and clears only on a power cycle or a TX-only init — so it looks like
black magic. Rule: do all non-I2C setup + `amy_start()` first, then `Wire.begin()`
and every I2C peripheral after. The official `AMY_custom_dsp` example follows this
(it touches no I2C before `amy_start()`), which is why it cold-boots clean.

---

## 3. The dual-core pattern (use it whenever you have slow I2C + audio)

ESP32-S3 has two cores. Slow blocking work (OLED flush ~100 ms, big I2C bursts)
must NOT share a core with the audio loop.

```c
void uiTask(void* arg) {
    for (;;) {
        // pots, LEDs, OLED flush, MIDI parse+send — ALL I2C lives here
        vTaskDelay(pdMS_TO_TICKS(2));   // feed watchdog / bound the rate
    }
}
// setup():
xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, NULL, 0);  // core 0
// loop() (core 1) = audio / time-critical only, touches NO I2C
```

**Rules that prevent the worst bugs:**
1. **One task owns the I2C bus.** Never touch Wire from both cores. With a single
   owner you need no mutex and you avoid intermittent bus-wedge races.
2. **Shared state = `volatile` + word-sized** (float/int/bool) -> atomic on
   ESP32, no mutex. Multi-word structs need protection; avoid sharing them.
3. **Serial from one core ideally.** Two cores doing `Serial.printf` can interleave
   /garble. If you must, keep it rare and short.
4. **MIDI TX belongs on the UI core too** — emit CCs from the same task that reads
   the pots, so the read → compare → send path is single-threaded and never races
   the values it's transmitting.

---

## 4. I2C peripheral cookbook

Generic register-style I2C device (8Angle, ADS1015, OLED all follow this shape):

```c
// write register pointer, read N bytes:
Wire.beginTransmission(ADDR);
Wire.write(reg);
Wire.endTransmission(false);          // repeated-start; keep bus for the read
Wire.requestFrom(ADDR, N);
while (Wire.available()) { ... }
```

- **M5Stack 8Angle (`0x43`):** pot ch -> reg `ch*2`, 2 bytes LE, 12-bit. Toggle
  -> reg `0x20` bit0. LED -> reg `0x30+ch*4` then R,G,B,bright.
- **SH1106/SSD1306 OLED (`0x3C`):** command stream prefix `0x00`, data stream
  prefix `0x40`. Chunk data (Arduino Wire TX buffer is small — write ~16 data
  bytes per transaction). Full 128x64 flush ~100 ms at 100 kHz -> keep on UI core.
- **ADS1015/1115 (CV ADC):** config register sets mux/gain/rate; conversion
  register holds the result. Good for reading CV/control voltages.

### Orientation flags for rotary controls (generalises beyond 8Angle)
Keep **channel order** (which knob is which) and **turn direction** (CW up vs
down) as **two independent flags**. Mounting the board rotated changes the
former, never the latter. Direction is electrical: `v = MAX - v`.

### I2C bus speed
Default 100 kHz is conservative (chosen for the slowest device, e.g. the 8Angle's
STM32). Bumping to 400 kHz (`Wire.setClock(400000)`) cuts a ~100 ms OLED flush to
~25 ms if every device on the bus tolerates it — verify each datasheet first.

---

## 5. Real-time MIDI (read this before you "just add" it)

A basic MIDI-IN UART is easy; **real-time/clock-tight MIDI is not.** Know which
you're building.

### Basic MIDI IN (works, low effort)
```c
Serial1.begin(31250, SERIAL_8N1, AMYBOARD_MIDI_IN, -1);  // RX only
// in uiTask: read 3-byte channel messages, dispatch CCs/notes
```
- 31250 baud, GPIO 21, the physical layer is the fiddly part — but once a single
  CC gets through, your wiring/polarity is proven. Adding more CCs is then just
  more `if (cc == X)` cases.
- **MIDI IN physical layer is polarity-agnostic on this board.** The input has a
  **bridge rectifier ahead of the H11L1 optocoupler**, so it accepts either TRS
  Type-A or Type-B without a jumper — don't burn time chasing input polarity.
- **Resolution caveat:** CC value is 7-bit (0-127). Mapping to a wide parameter
  range is coarse (128 steps). Use 14-bit (CC MSB+LSB) or pitch-bend for smooth.

### MIDI OUT on the AMYboard — the differential-drive gotcha (this is the big one)

MIDI OUT is **NOT a single-pin UART TX** on this board. It routes two GPIOs to the
two TRS out contacts, each through a 22 Ω series resistor:
`AMYBOARD_MIDI_OUT_TYPE_A = 14` (→ tip → DIN pin 5) and
`AMYBOARD_MIDI_OUT_TYPE_B = 15` (→ ring → DIN pin 4). A standard DIN MIDI input is
a **current loop** (pins 4–5) and needs BOTH legs actively defined.

**Measured truth table** (sending CC into a MOTU M2 — a deliberately picky
receiver; if it's clean there it's clean anywhere):

| TX pin | Other pin | Result |
|--------|-----------|--------|
| 14 | 15 floating | silence (return leg undefined) |
| 14 | 15 driven LOW | silence |
| 15 | 14 floating | silence |
| 15 | 14 driven LOW | **garbled** (loop closes, but static-LOW return is marginal → dirty edges → receiver mis-frames status bytes) |

A static-LOW return *closes* the loop but drives it so marginally (3.3 V, 22 Ω,
single-ended) that a picky opto input mis-samples bit edges — the monitor shows
scattered Program/Note/PitchWheel messages on random channels while the ramping
value byte leaks through recognizably. Do not ship this.

**Correct fix — differential push-pull drive.** Drive UART1 TX **normal on GPIO 14**
and its **hardware-inverted mirror on GPIO 15** via the ESP32-S3 GPIO matrix:

```c
#include "esp_rom_gpio.h"
#include "soc/uart_periph.h"
#include "hal/gpio_hal.h"

Serial1.begin(31250, SERIAL_8N1, AMYBOARD_MIDI_IN, 14);  // TX normal on 14
Serial1.setPins(AMYBOARD_MIDI_IN, 14);
uint32_t tx_sig = uart_periph_signal[1].pins[SOC_UART_TX_PIN_IDX].signal; // UART1
gpio_set_direction((gpio_num_t)15, GPIO_MODE_OUTPUT);
esp_rom_gpio_connect_out_signal((gpio_num_t)15, tx_sig, true /*invert*/, false);
```

The opto then sees a full push-pull differential swing with clean, bit-aligned
edges (both pads emit the same UART1 TX signal in hardware, one inverted, zero
CPU, no skew). **Orientation matters: TX on 14, inverted mirror on 15 — the
reverse garbles.** This is how the board drives a 5 V-era MIDI input cleanly from
3.3 V logic (differential ≈ 2× headroom).

**Why AMY's own code doesn't reveal this:** AMY's `esp_init_midi` does a plain
single-ended `uart_set_pin(uart, midi_out, midi_in, ...)` on ONE pin, and nothing
in the tree ever drives the second contact. Single-ended is fine for a **TRS-native
Type-A receiver** (the return is defined by the TRS standard), but NOT for a
**DIN-4/5 loop via a TRS→DIN adapter**, which is why you must drive the return
actively yourself. Selecting Type-A vs Type-B in AMY just changes *which* single
pin carries data; it never sets up the differential drive.

**Debug path that cracked it (generalises):** scope the ESP pin (does TX leave the
chip?), then the jack (does the board pass it?), then the DIN pins (did data reach
pin 4/5, and on which contact?). This walks the signal outward and localizes the
fault to a *layer* — here, the loop's return leg — instead of guessing. The `.ino`
code was correct the whole time; the fault was purely the output electrical layer.

### MIDI TX logic (knob → CC), once the electrical layer works
- **Physical-move-only, never echo RX.** Send only inside the pot
  `any_changed`/deadband path, and skip any parameter currently driven by an
  incoming CC (`s_midiOwned[]`) so RX→TX can't feedback-loop.
- **Deadband = 7-bit re-quantization.** Emit only when the outgoing 0–127 value
  actually changes; a caught pot resting under a finger then never spews CCs.
- **Prime a baseline.** On the first pass capture current positions and send
  nothing, so touching one knob doesn't blast all eight.
- **Send raw knob position** (e.g. `pot_raw>>5`) if RX applies the same taper on
  the way back in — that makes a DAW round-trip a knob to the same sound.

### Why a basic parser is NOT clock-ready
1. **Timing/jitter, not parsing, is the problem.** If MIDI is serviced in the UI
   task that also runs the ~100 ms OLED flush, bytes queue during the flush and
   get processed in a burst afterwards -> up to ~100 ms of jitter. MIDI clock
   (24 PPQN, one byte every ~20 ms at 120 BPM) becomes lurching/unusable.
2. **Real-time messages are single bytes** (clock `0xF8`, start/stop/continue)
   that can appear ANYWHERE, even mid-message. A fixed "wait for 3 bytes" parser
   mis-frames on them.

### Doing clock properly (when you commit to it)
- **Timestamp `0xF8` as close to the wire as possible** — UART ISR / driver event
  queue, NOT in a task that can be blocked by the OLED. This is the single biggest
  lever against jitter.
- **Byte-at-a-time state-machine parser** with running-status support that
  dispatches single-byte real-time messages immediately, separate from channel-
  message assembly.
- **Tempo recovery is inherently noisy** (24 PPQN is a coarse estimate). Everyone
  picks a different smoothing filter (PLL / moving average); that's WHY device A
  syncs tight to B but wobbles with C. Choose your tempo-filter tradeoff
  consciously; don't inherit it by accident.
- **No phase reference in clock alone** — use Start/Continue for the downbeat
  anchor.

Bottom line: occasional set-once CCs are a trivial add to a basic parser;
real-time clock is a deliberate feature needing its own ISR-timestamped,
state-machine architecture. Don't bolt clock onto a display-blocked task.

---

## 6. Debugging methodology (project-agnostic, earned the hard way)

- **Build markers.** Print `Ready [BUILD: rev-X]` on boot; confirm on-device that
  the running binary matches the edited file. Tooling/flash desync silently
  invalidates "fixes."
- **Measure, never assume.** Dump raw values; scope real outputs. A matching
  envelope is not a matching waveform. (The MIDI-out fix came from scoping the
  pin → jack → DIN pins, not from re-reading the code.)
- **Walk the signal outward to localize a fault to a layer.** For any output
  chain (MIDI, audio, CV), probe at successive physical points — chip pin, board
  connector, cable far end — and the first point where it breaks names the
  culprit layer. Saves you auditing correct code.
- **Isolate with minimal sketches** to partition a problem (is it the DAC path,
  the input path, or my code?). One bare sketch beats hours of reasoning about a
  big one. (A ~40-line heartbeat sketch that just spammed a CC out is what let us
  build the MIDI-out truth table cleanly, away from the full firmware.)
- **Amplitude sweep** for "output looks wrong": drive 0.8/0.1/0.01.
  Same shape = sign/format/MSB bug; scales = gain/clip. Opposite fixes.
- **Read a garbled-vs-silent distinction as a real clue.** *Nothing* arriving =
  the physical loop isn't conducting (open leg, wrong pin). *Garbage* arriving =
  bytes conduct but edges/levels are marginal (mis-sampling). They point at
  different layers and different fixes.
- **Trust source over inference for this board.** Repeatedly, reasoning from the
  schematic or the MIDI spec pointed the wrong way (input "polarity", which pin is
  "Type-A", single-vs-differential); the AMY source and direct measurement settled
  each one. When they disagree, believe the measurement.
- **Disproven theories are progress** — they shrink the search space. Don't loop
  back to a killed hypothesis without new evidence.
- **Single-owner discipline** for shared resources (I2C bus, Serial) prevents the
  hardest-to-reproduce class of bugs.

---

## 9. Toolchain version pinning (do not skip)

AMY's I2S layer is **tightly coupled to the arduino-esp32 core / ESP-IDF
version.** AMY's maintainers actively patch `src/i2s.c` to track board-package
changes, which means a board update can require a matching AMY update — and a
mismatch breaks audio subtly.

**Pin the known-good pairing and record it in your project:**
- AMY documents its tested combo in `docs/arduino.md` (e.g. "arduino-esp32
  3.2.0"). Use that core version with the matching AMY release.
- Updating arduino-esp32 to a newer core without updating AMY can break the
  **full-duplex audio-in (I2S RX)** path while leaving **synth-out (I2S TX)**
  working — a confusing, misleading failure that looks like a code regression.

**Diagnostic signature of a core/AMY version skew:**
- Bare "AMY plays a sine" sketch: clean (TX works).
- Audio in -> out: loud/distorted/level-independent (RX path broken).
- Your code unchanged, build marker correct, digital output samples clean.

**Fix:** roll the core back to the AMY-tested version (Boards Manager -> esp32 ->
version dropdown), OR update AMY in lockstep to a release that supports the newer
core. Change ONE thing at a time and re-validate audio-in with a minimal
passthrough before touching your sketch. The sketch is almost never the cause of
this particular failure.

**⚠️ Same symptom, different cause — disambiguate first.** "Loud/distorted/
level-independent audio-in with clean digital samples" has (at least) TWO root
causes:
- **Version skew (this section):** the *bare* audio-in passthrough example also
  breaks; independent of init order; fixed by aligning (core, AMY) versions.
- **I2C-before-`amy_start()` (a setup-ordering bug):** the bare example **works**;
  only your sketch breaks; survives reflash/reset but a power cycle re-breaks it;
  fixed by calling `amy_start()` before any Wire/I2C (see the startup section).
Decisive test: flash the bare official example and power-cycle it. Clean → it's
your init order, not versions. Broken → versions.

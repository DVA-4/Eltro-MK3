/**
 * eltro_amyboard.ino  (V12)
 * Eltro MK3 "Information Rate Changer" — AMYboard + M5Stack 8Angle + SH1106
 * Analog pass-through: AUDIO IN → Eltro DSP → AUDIO OUT
 *
 * V12 change vs V11: Page-A knobs now use PICKUP (catch) mode on page RE-ENTRY,
 * mirroring Page B. Previously, returning to Page A snapped every Page-A parameter
 * to its physical pot position (a jump). Now, when Page A is (re)entered, all eight
 * knobs are armed "waiting" at their current reading and a knob does not drive its
 * parameter until moved past PAGEA_PICKUP_THRESH — so values held while you were on
 * Page B (or set by MIDI) are preserved until you deliberately grab a knob.
 * BOOT EXCEPTION: at power-on Page A is LIVE (knobs take their physical position
 * immediately), matching the familiar "panel is live at startup" feel; pickup only
 * applies on subsequent re-entries. Page B is unchanged (it pickups on boot too,
 * because its values are persisted). See rearmPageACatch() / potDrivesPageA().
 *
 * V11 change vs V10: MIDI TX added (knob → CC out). The eight Page-A knobs emit
 * their CCs (102–109) when PHYSICALLY moved, so a DAW/sequencer can record and
 * play back knob motion. Two guards prevent an RX/TX feedback loop: TX runs only
 * on real pot movement (the existing POT_DEADBAND / any_changed path), and skips
 * any knob currently MIDI-owned (s_midiOwned[]) so an incoming CC is never echoed
 * back out. Deadband = 7-bit re-quantization (emit only when the 0..127 value
 * changes). Serial1 now inits with a TX pin (was -1). TX pin is a single #define
 * (ELTRO_MIDI_TX_PIN) — the loop is driven DIFFERENTIALLY (TX normal on 14, its
 * hardware-inverted mirror on 15) so a DIN-4/5 opto input receives cleanly; see
 * the note there.
 * Everything else is byte-identical to V10; DSP/ordering/phase-lock untouched.
 * V10 kept as the stable fallback.
 *
 * V10 change vs V9: the four read heads are now PHASE-LOCKED to a single master
 * scan phase (each head derived as master + (arcLen/NUM_HEADS)*h) instead of four
 * independent scan accumulators. This fixes a bug where changing the buffer (or
 * segment) length at runtime permanently collapsed the head stagger — the heads
 * bunched up and produced a stutter that survived a full buffer rewrite. Steady-
 * state sound is identical to V9 (verified off-board); only the buffer-change
 * robustness changed. V9 kept as the stable fallback.
 *
 * Hardware:
 *   AMYboard (ESP32-S3, 44.1kHz / 16-bit stereo / 256-sample blocks)
 *   M5Stack 8Angle (I2C 0x43) on AMYboard FRONT-panel Grove I2C (SDA=17, SCL=18)
 *   SH1106 128×64 OLED (I2C 0x3C) on same Grove bus
 *   CV1 jack → tape axis, CV2 jack → drum axis; both read via ADS1115 (I2C 0x48, A0/A1)
 *  
 * AUDIO ARCHITECTURE — insert effect via amy_external_render_hook:
 *   AMY calls the render hook once per audible oscillator BEFORE the block
 *   is summed and sent to I2S. We instantiate one oscillator (osc 0) as
 *   the external-buffer path (AUDIO_EXT0/1). Input is pulled with
 *   stereo) holds the live ADC input; we mix to mono, run the Eltro DSP,
 *   and write the mono result into AMY's per-osc SAMPLE buffer, returning 1
 *   ("handled") so AMY mixes our output to AUDIO OUT.
 *
 *   NOTE: write_samples_fn is NOT used for processing — it is a downstream
 *   tap that runs after I2S TX and cannot alter the output.
 *
 * Physics model (V8 — faithful Tempophon, TWO INDEPENDENT AXES):
 *   The buffer is a pre-written tape: one write head, constant +1.0, unit density,
 *   independent of everything (tape speed does NOT touch the write).
 *   Reading is split into the two mechanisms the real machine uses to decouple
 *   time from pitch:
 *     PROGRESSION (time): a grab playhead walks through the buffered tape at the
 *       TAPE rate. How fast you move through the recording. Depends on tape ONLY.
 *     PITCH: within each grabbed segment, the read scans the material at the
 *       RELATIVE velocity (tape − drum). Re-pitches without changing progression.
 *   Each head grabs a fresh segment at the (tape-advanced) grab playhead, scans it
 *   at (tape − drum), and hands off to the next head with a Hann crossfade.
 *   Segments repeat/drop to reconcile the two rates — Carlos's "itty-bitty bits,
 *   cross-faded." Consequences (all validated off-board):
 *       tape == drum      → scan frozen → SILENCE/FREEZE
 *       tape − drum == 1  → scans at write density → recorded pitch (unity)
 *       drum  > tape      → reverse scan within the grain
 *       tape fixed, drum varied → SPEED HOLDS, PITCH MOVES (the decoupling)
 *       tape == 0         → grab frozen → freeze on a spot
 *   This is a grain-based engine (V7 was a single continuous scan): segment length
 *   (Page-B K2) and overlap (Page-A K2) are now the key tone controls. Real-time
 *   single-pass ⇒ the decoupling operates over BUFFERED tape (up to ~20 s of
 *   recorded history), not a live stream — we can re-scan the past, not the future.
 *
 * Head geometry (V8 — grain engine, not a physical hard-switch):
 *   The original drum has 4 heads at 90° spacing with one always in contact,
 *   so the real machine hands off near-instantaneously. V8 does NOT model that
 *   as a hard switch: the two-axis grain engine reconciles progression and pitch
 *   by repeating/dropping segments, and that handoff MUST be crossfaded or the
 *   seams click. Overlap (Page-A K2) is therefore a real tone control, not a
 *   near-zero authenticity detail: range 0.10–0.95, default 0.25. Below ~0.10
 *   the granular seams become audible. Higher values smooth the grain further.
 *
 * Decay (erase):
 *   ring[wi] = input*(1-decay) + ring[wi]*decay
 *   0.0 = full erase (normal pass), ~1.0 = infinite loop accumulation
 *
 * Wow/Flutter combined:
 *   wow = K3 value, flutter = wow * 0.6 (fixed ratio, shallower)
 *
 * Page A (slider OFF, LEDs blue) — main controls:
 *   K0  Tape speed    −range – +range  (centre detent = 0 = stopped; range = Page-B K5)
 *   K1  Drum speed    −range – +range  (centre detent = 0 = stopped; range = Page-B K6)
 *                     Both knobs use an exponential taper (g_speedGamma, default
 *                     2.0) so the dense −1..+1 region gets more travel; set live
 *                     via serial 'g' (e.g. "g2.5"), persisted to NVS.
 *   K2  Head overlap   0.10 – 0.95  (default 0.25; below ~0.10 seams click)
 *   K3  Wow/Flutter    0.0  – 0.05
 *   K4  Input gain     0.0  – 4.0   (raised from 2.0 for the quiet AMYboard input)
 *   K5  Output gain    0.0  – 1.5
 *   K6  Decay          0.0  – 0.9999
 *   K7  Buffer length  512  – 884736 samples (~12 ms – 20 s, log scale)
 *   Toggle (latching slider) → bypass OFF/ON (dry passthrough; the tape keeps
 *                              recording in bypass — decay forced 0 — so un-
 *                              bypassing resumes on live material, not a freeze)
 *
 * Page B (via Serial 'p' or MIDI CC 65) — config:
 *   K0  CV1 scale     0.0 – 4.0   (CV-in 1 → TAPE axis)
 *   K1  CV1 offset   −2.0 – +2.0  (CV-in 1 → TAPE axis)
 *   K2  CV2 scale     0.0 – 4.0   (CV-in 2 → DRUM axis)
 *   K3  CV2 offset   −2.0 – +2.0  (CV-in 2 → DRUM axis)
 *   K4  Segment len    64 – 32768 samples (~1.5–743 ms) — absolute, independent
 *                       of buffer; short=textured/buzzy, long=smooth/fluid
 *                       (default 4096 ≈93ms; lower=shorter/textured, higher=longer)
 *   K5  Tape range    1.0 – 5.0   — full-scale of Page-A tape knob (±)
 *   K6  Drum range    1.0 – 5.0   — full-scale of Page-A drum knob (±)
 *   K7  MIDI channel  1 – 16      — RX filter (and future TX channel)
 *
 * MIDI RX (V9): byte-at-a-time parser (running-status aware, real-time bytes
 *   skipped). Fixed CC map on the Page-B K7 channel, all 7-bit, each mapped
 *   through the SAME law as its knob (so CC == equivalent knob position):
 *     CC102=tape  CC103=drum  CC104=overlap  CC105=wow  CC106=in gain
 *     CC107=out gain  CC108=decay  CC109=buffer   (CC65 = page toggle)
 *   Per-knob "catch" takeover: a CC owns its knob until the pot moves past
 *   MIDI_CATCH_THRESH (60/4095); OLED shows '>' by MIDI-owned Page-A knobs.
 *   TX (knob→CC, deadband) is a planned stage 2; jack wired, code still RX-only.
 *
 * Serial commands: 'p' = toggle Page A/B; 'g<value>' = set speed-knob taper
 *   gamma (1.0–4.0, e.g. "g2.5"; bare 'g' prints current). Both persist to NVS.
 *
 * USB SERIAL NOTE:
 *   The AMYboard's board definition does not set ARDUINO_USB_CDC_ON_BOOT, and
 *   exposes no "USB CDC On Boot" Tools menu option to enable it. Without this,
 *   plain Serial never attaches to the native USB peripheral — confirmed by
 *   testing (zero output, in any terminal app, with no errors) and by tracing
 *   Espressif's usb_cdc.h docs, which show this exact scenario requires a
 *   manually instantiated USBCDC object + USB.begin(). We alias Serial to that
 *   object via #define so the rest of the sketch needs no further changes.
 *   Serial1 (TRS MIDI UART, separate hardware) is unaffected by this.
 *
 * Libraries needed (Arduino Library Manager):
 *   AMY (Arduino Library Manager). No display library — the SH1106 OLED
 *   is driven by a small built-in direct-I2C driver (see eltro_font5x7.h).
 */

#include <AMY-Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Preferences.h>   // ESP32 NVS flash storage for persistent Page-B settings
#include "esp_heap_caps.h" // PSRAM allocation for the ring buffer (V5 test)
#include "USB.h"
#include "esp_rom_gpio.h"     // esp_rom_gpio_connect_out_signal — MIDI TX inverted mirror
#include "soc/uart_periph.h"  // uart_periph_signal[] — UART1 TX signal index
#include "hal/gpio_hal.h"     // gpio_set_direction

// AMYboard's board definition does not set ARDUINO_USB_CDC_ON_BOOT, so plain
// Serial never attaches to the native USB peripheral on this board — confirmed
// by tracing Espressif's usb_cdc docs and testing. We must instantiate the CDC
// interface manually and call USB.begin() ourselves. Serial1 (TRS MIDI UART)
// is a separate peripheral and is unaffected by this.
#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#define Serial USBSerial
#endif

// ═══════════════════════════════════════════════════════════════════════════════
// 1.  CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════════

#define SAMPLE_RATE        AMY_SAMPLE_RATE
#define BLOCK_SIZE         AMY_BLOCK_SIZE
#define NUM_HEADS          4
// V5 PSRAM TEST: ring buffer in PSRAM (8MB on this board), 5 s. This re-tests
// PSRAM now that the O(bufLen/segLen) wrapf cost is fixed — the earlier "PSRAM
// too slow" verdict was contaminated by that loop bug. 221184 = 5.01 s @44.1k,
// divisible by NUM_HEADS and 2. Minimal change: only the buffer moved, no DSP
// reorder. If this holds 172/s, PSRAM is viable and we can go larger.
// Ring buffer in PSRAM (8MB on this board), 20 s. PSRAM works fine for the audio
// buffer once the per-sample wrapf cost was fixed (the earlier "PSRAM too slow"
// verdict was that loop bug, not memory latency). 884736 = 20.06 s @44.1k,
// divisible by NUM_HEADS and 2. Uses ~1.69 MB of the 8 MB PSRAM. Confirmed on
// hardware: holds 172 blocks/s across all buffer and segment sizes.
// Note (V7): read heads roam the whole buffer freely by (tape−drum); they start
// at bufLen/2 once at init, then move independently. The buffer takes ~20 s to
// fully populate from cold, so there is a startup-fill transient at large buffers.
#define RING_BUF_MAX       884736  // ~20.06 s at 44.1k, in PSRAM
#define RING_BUF_MIN       512

// Segment length is ABSOLUTE (a fixed sample count), independent of buffer length,
// so changing the buffer doesn't disturb the grain character. Set on Page B K2,
// linear in samples. ~1.5 ms (granular/buzzy, by choice) up to ~743 ms (smooth).
// Default ~4096 = 93 ms = the v0.x character.
#define SEG_MIN            64       // ~1.5 ms — granular/buzzy (intentionally reachable)
#define SEG_MAX            32768    // ~743 ms — long/smooth
#define SEG_DEFAULT        4096     // ~93 ms — matches original v0.x feel

#define ANGLE8_ADDR        0x43
#define ANGLE8_POTS        8

// ── CV input: ADS1115 16-bit ADC ────────────────────────────────────────────
// Official AMYboard CV-in hardware (tulipcc docs/amyboard/modular.md): two CV
// input jacks on an ADS1115 at I2C 0x48, accepting ±10 V at the jack. It sits on
// the same front Grove bus the uiTask already owns, so no extra bus/mutex.
// We read CV-in 1 (A0) → g_cv1Value and CV-in 2 (A1) → g_cv2Value, each mapped
// 1 unit per volt and clamped to ±5.0 (the musical reference swing). They fold
// into the two axes in eltroProcess via:
//   effectiveTape = tape + cv1Value*cv1Scale + cv1Offset   (CV1 → tape, faithful)
//   effectiveDrum = drum + cv2Value*cv2Scale + cv2Offset   (CV2 → drum)
// CV1 acts as a remote tape knob: effectiveTape is used everywhere tapeSpeed is
// (both progression AND the tape−drum pitch term), so CV1 and the tape knob
// behave identically.
#define ADS1115_ADDR       0x48
#define ADS1115_CH_CV1     0       // CV-in 1 = single-ended A0 (→ tape)
#define ADS1115_CH_CV2     1       // CV-in 2 = single-ended A1 (→ drum)
#define CV_READ_MS         10      // throttle EACH channel's read to ~100 Hz in uiTask
#define CV_CLAMP           5.0f    // |cvValue| ceiling (±5 V reference swing)
#define CV_EMA_ALPHA       0.25f   // light smoothing on the CV read (0=frozen,1=raw)
// Volts-per-count for the ADS1115 at the ±6.144 V PGA setting (FS = 2^15 counts).
// CALIBRATION: this assumes the board's front-end presents the CV within the
// ADC's ±6.144 V window. If a known input voltage reads wrong, this is the ONE
// constant to trim — measured_volts / raw_count. (±6.144V / 32768 ≈ 187.5 µV.)
#define ADS1115_V_PER_COUNT (6.144f / 32768.0f)

// AMYboard front-panel Grove I2C bus (accessories). NOT the back "Tulip" host port.
// Source: AMYboard accessories docs — SDA=17, SCL=18, 400kHz.
#define I2C_SDA_PIN        17
#define I2C_SCL_PIN        18
#define I2C_SCAN_MS        12
#define POT_DEADBAND       10
#define LED_FLASH_MS       60

// ── V9 MIDI: fixed CC map for Page-A knobs (RX; TX added later) ───────────────
// Eight contiguous CCs in the "undefined" range 102–119 (reserved by the MIDI
// spec for exactly this, so they won't collide with standard controllers like
// mod=1, volume=7, expression=11, sustain=64, or the mode messages 120–127).
// CC65 stays the page toggle (unchanged). Each CC drives the same parameter as
// the matching Page-A pot, mapped through the SAME law (taper for tape/drum, log
// for buffer) so a CC value and the equivalent knob position produce identical
// results — MIDI and hand are interchangeable.
#define MIDI_CC_TAPE    102
#define MIDI_CC_DRUM    103
#define MIDI_CC_OVERLAP 104
#define MIDI_CC_WOW     105
#define MIDI_CC_INGAIN  106
#define MIDI_CC_OUTGAIN 107
#define MIDI_CC_DECAY   108
#define MIDI_CC_BUFLEN  109
#define MIDI_CC_PAGE    65        // existing page-toggle CC (Page A/B)

// ── V11 MIDI TX pins (differential drive) ────────────────────────────────────
// MIDI IN is on AMYBOARD_MIDI_IN (GPIO 21). MIDI OUT is a TRS current loop the
// AMYboard brings to TWO GPIOs, one per contact (src/amy.h):
//     #define AMYBOARD_MIDI_OUT_TYPE_A 14   // -> tip -> DIN pin 5
//     #define AMYBOARD_MIDI_OUT_TYPE_B 15   // -> ring -> DIN pin 4
// A DIN-4/5 opto input needs BOTH loop legs actively defined. Single-ended (one
// pin driven, other floating) never closes the loop -> silence. A static-LOW
// return closes it but with marginal levels/edges -> the byte stream arrives
// GARBLED (a picky interface like the MOTU M2 mis-samples it).
// BENCH-CONFIRMED FIX (clean CC102 into the M2): drive the loop DIFFERENTIALLY —
//   GPIO14 = UART1 TX, NORMAL   (data / source leg)
//   GPIO15 = UART1 TX, INVERTED (active return leg, via the ESP32-S3 GPIO matrix)
// The receiver opto then sees a full push-pull swing with clean, bit-aligned
// edges. Orientation matters: TX must be on 14 and the inverse on 15 (the reverse
// garbles). This is set up in setup(); see the esp_rom_gpio_connect_out_signal()
// call. RX is unaffected.
#define ELTRO_MIDI_TX_PIN     14   // UART1 TX, normal  — data leg (tip, DIN pin 5)
#define ELTRO_MIDI_RETURN_PIN 15   // UART1 TX, INVERTED — active return (ring, DIN pin 4)

// Per-knob catch takeover on Page A: an incoming CC owns the parameter until the
// physical pot is moved past this raw-ADC threshold (of 4095). Larger than the
// Page-B value (40) because these are live-performance knobs a hand rests on —
// a small accidental brush shouldn't yank a MIDI-automated value back.
#define MIDI_CATCH_THRESH 60

#define DISPLAY_MS         100   // 10 fps. Display runs on core 0 (uiTask), so the
                                 // ~100ms OLED flush no longer affects audio.

// ═══════════════════════════════════════════════════════════════════════════════
// 2.  SHARED STATE
// ═══════════════════════════════════════════════════════════════════════════════

volatile float    g_tapeSpeed   =  1.0f;   // tape transport speed (the recorded tape moving past the heads)
volatile float    g_drumSpeed   =  0.0f;   // drum (head rotor) speed

// Faithful Tempophon model (V6): the read-through speed = (tape − drum).
//   SILENCE when tape == drum (no relative motion — heads ride with the tape).
//   UNITY PITCH (plays at recorded pitch) when (tape − drum) == 1.
//   drum > tape → reverse scan; drum opposing tape → pitch up; etc.
// READTHRU_UNITY is the (tape − drum) value that reproduces original pitch.
#define READTHRU_UNITY (1.0f)
volatile float    g_overlap     =  0.25f;   // V8: granular needs real overlap to hide segment seams (V7 used 0.03 for continuous scan)
volatile float    g_wow         =  0.0f;
volatile float    g_inputGain   =  1.0f;
volatile float    g_outputGain  =  0.8f;
volatile float    g_decay       =  0.0f;
volatile uint32_t g_bufLen      =  RING_BUF_MAX;

// CV1 → tape axis, CV2 → drum axis. Scale/offset per channel (Page B K0/K1 for
// CV1, K2/K3 for CV2). Values driven by serviceCvInput() in uiTask.
volatile float    g_cv1Scale    =  1.0f;
volatile float    g_cv1Offset   =  0.0f;
volatile float    g_cv1Value    =  0.0f;   // ADS1115 A0, ±5 unit clamp → tape
volatile float    g_cv2Scale    =  1.0f;
volatile float    g_cv2Offset   =  0.0f;
volatile float    g_cv2Value    =  0.0f;   // ADS1115 A1, ±5 unit clamp → drum

// Speed-knob ranges (Page B K3/K4, each 1.0–5.0). Page-A tape and drum knobs are
// SYMMETRIC about a centre detent at 0: tape -range..+range, drum likewise.
// Centre (0) = stopped. Unity pitch = drum==tape; normal play at unison = both
// equal & positive. Larger range = wider throw.
volatile float    g_tapeRange   =  1.0f;
volatile float    g_drumRange   =  1.0f;

// Speed-knob taper (tape + drum). The bipolar tape/drum knobs are reshaped by an
// exponential curve so the musically dense −1..+1 region around centre gets more
// knob travel, instead of being crammed into a small fraction of the throw when
// the range is large. Curve: out = sign(x) * |x|^gamma * range, where x is the
// knob's signed position in [−1,+1]. gamma = 1 is the old linear mapping; gamma
// > 1 expands the centre (finer near 0), gamma < 1 would compress it. Persisted
// to NVS; set live over serial with the 'g' command (e.g. "g2.5"). Bounded to a
// sane range so a bad value can't make the knob unusable.
#define SPEED_GAMMA_MIN  1.0f
#define SPEED_GAMMA_MAX  4.0f
volatile float    g_speedGamma  =  2.0f;   // default: −1..+1 gets ~45% of travel (vs ~20% linear at range 5)

// V9 MIDI receive channel (1–16), set on Page B K7, persisted to NVS. Filters
// incoming channel-voice messages; will also select the TX channel when TX lands.
volatile uint8_t  g_midiChannel =  1;

// Segment length in SAMPLES (Page B K2, SEG_MIN..SEG_MAX, linear). ABSOLUTE —
// independent of buffer length, so the grain character no longer shifts when you
// change the buffer. The four heads space themselves segLen apart (clean tiling).
// ~64 samples (buzzy) .. ~32768 (smooth); default ~4096 (≈93 ms, v0.x feel).
volatile float    g_segLen      =  (float)SEG_DEFAULT;

volatile bool     g_bypass      =  false;
volatile bool     g_pageB       =  false;

volatile uint16_t g_pot_display[ANGLE8_POTS] = {0};
volatile uint8_t  g_last_moved_pot = 255;

// ═══════════════════════════════════════════════════════════════════════════════
// 3.  ELTRO DSP ENGINE
// ═══════════════════════════════════════════════════════════════════════════════

// V5: ring buffer in PSRAM (allocated at boot). Pointer + capacity; ringAlloc()
// fills it before any audio. Falls back to a small SRAM buffer if PSRAM fails.
static int16_t* s_ring         = nullptr;
static uint32_t s_ringCap      = 0;
static float    s_writePtrF    = 0.0f;
// V8 two-axis Tempophon state. The single read position (V7's s_headPos) is split
// into two independent mechanisms, reproducing how the real machine decouples
// time from pitch:
//   s_grab        : a shared "grab playhead" that walks through the buffered tape
//                   at the TAPE rate. This is the PROGRESSION axis (how fast we
//                   move through the recording). Depends on tape ONLY, not drum.
//   s_segOrigin[] : where each head grabbed its current segment from (a snapshot
//                   of s_grab taken at that head's segment boundary).
//   s_masterScan  : ONE scan phase advancing at the relative velocity (tape −
//                   drum) — the PITCH axis. Each head's read offset within its
//                   segment is DERIVED as (s_masterScan + (arcLen/NUM_HEADS)*h),
//                   so the four heads are evenly staggered by construction and
//                   cannot drift/bunch when the buffer or segment length changes.
// read address = s_segOrigin[h] + derived-scan. Segments repeat/drop to reconcile
// the two rates; the Hann crossfade hides the handoff. This is the mechanism, not
// a textbook stretcher — validated off-board (progression = tape·SR independent of
// drum; output pitch = f·(tape−drum) independent of progression).
static float    s_grab         = 0.0f;
static float    s_segOrigin[NUM_HEADS];
// V10: heads are PHASE-LOCKED. Instead of NUM_HEADS independently free-running
// scan accumulators (which drift apart and permanently collapse the stagger when
// the buffer/segment length changes at runtime — the "stutter that survives a
// full buffer rewrite" bug), we keep ONE master scan phase and derive each head's
// position as wrap(master + (arcLen/NUM_HEADS)*h, arcLen). The even stagger is
// then true BY CONSTRUCTION at any buffer/segment length and cannot bunch up.
// Each head still captures its OWN segment origin at its OWN boundary crossing,
// preserving the multi-grab texture. s_prevSegPhase[h] tracks each head's derived
// phase across samples so we can detect its individual segment-boundary wrap.
static float    s_masterScan   = 0.0f;
static float    s_prevSegPhase[NUM_HEADS];
static float    s_wowPhase     = 0.0f;
static float    s_flutterPhase = 0.0f;
// (mono scratch is a local in eltroProcessBlock, not a shared global)

#define RING_FALLBACK 65536
static int16_t  s_ringFallback[RING_FALLBACK];

static void ringAlloc() {
    s_ring = (int16_t*)heap_caps_malloc((size_t)RING_BUF_MAX * sizeof(int16_t),
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_ring) {
        s_ringCap = RING_BUF_MAX;
    } else {
        s_ring    = s_ringFallback;
        s_ringCap = RING_FALLBACK;
    }
    memset(s_ring, 0, (size_t)s_ringCap * sizeof(int16_t));
}

// DSP diagnostics (written in eltroProcess, read from loop diag print)
volatile float g_dbgOutputGain = 0;
volatile float g_dbgInputGain  = 0;
volatile float g_dbgNorm       = 0;
volatile float g_dbgTape       = 0;
volatile float g_dbgDrum       = 0;

// Hann window lookup table (kills the per-sample cosf in the hot path).
// 256 entries over t=[0,1]; linear interpolation between entries is overkill
// for a crossfade window, nearest is fine.
#define HANN_LUT_SIZE 256
static float s_hannLut[HANN_LUT_SIZE];
static void hannLutInit() {
    for (int i = 0; i < HANN_LUT_SIZE; i++)
        s_hannLut[i] = 0.5f * (1.0f - cosf(M_PI * 2.0f * (float)i / (float)(HANN_LUT_SIZE - 1)));
}
static inline float hannWindow(float t) {
    if (t <= 0.f) return s_hannLut[0];
    if (t >= 1.f) return s_hannLut[HANN_LUT_SIZE - 1];
    return s_hannLut[(int)(t * (HANN_LUT_SIZE - 1))];
}

// Fast wrap into [0, bufLen): the value is always within a few buffer-lengths
// of range, so a short conditional loop beats fmodf (which is very slow on the
// ESP32-S3 FPU path). Used in the audio hot loop.
static inline float wrapf(float x, float bufLen) {
    while (x >= bufLen) x -= bufLen;
    while (x < 0.f)     x += bufLen;
    return x;
}

static inline float ringRead(float pos, uint32_t bufLen) {
    pos = wrapf(pos, (float)bufLen);
    uint32_t i0   = (uint32_t)pos;
    uint32_t i1   = i0 + 1; if (i1 >= bufLen) i1 = 0;
    float    frac = pos - (float)i0;
    return s_ring[i0] * (1.0f - frac) + s_ring[i1] * frac;
}

// Processes mono[BLOCK_SIZE] in place (caller-owned buffer; values ±32768 float)
static void eltroProcess(float* mono) {
    if (g_bypass) {
        // Bypass: keep the tape LIVE so toggling the effect back on gives fresh
        // material instead of a frozen snapshot. We run ONLY the write head —
        // record the input (with input gain so the tape level matches the active
        // path) and advance the write pointer — then skip all read/grain work and
        // leave `mono` untouched so eltroProcessBlock passes the dry signal through.
        // Decay is forced to 0 here: a clean record with no feedback accumulation
        // while bypassed (we're not monitoring the loop, so don't let it build up).
        float    inputGain = g_inputGain;
        uint32_t bufLen    = g_bufLen;
        if (bufLen > s_ringCap) bufLen = s_ringCap;
        const float fbufLen = (float)bufLen;
        s_writePtrF = wrapf(s_writePtrF, fbufLen);
        for (int n = 0; n < BLOCK_SIZE; n++) {
            float inMono = mono[n] * inputGain;
            uint32_t wi  = (uint32_t)s_writePtrF;
            s_ring[wi]   = (int16_t)fmaxf(-32768.f, fminf(32767.f, inMono));  // decay = 0
            s_writePtrF += 1.0f;
            if (s_writePtrF >= fbufLen) s_writePtrF -= fbufLen;
        }
        return;   // mono left as-is → dry passthrough
    }

    float    tapeSpeed  = g_tapeSpeed;
    float    drumSpeed  = g_drumSpeed;
    float    overlap    = g_overlap;
    float    wow        = g_wow;
    float    flutter    = wow * 0.6f;
    float    inputGain  = g_inputGain;
    float    outputGain = g_outputGain;
    float    decay      = g_decay;
    uint32_t bufLen     = g_bufLen;
    if (bufLen > s_ringCap) bufLen = s_ringCap;   // never exceed allocated ring
    // CV1 → tape (used everywhere tapeSpeed is, so CV1 acts as a remote tape knob:
    // it shifts BOTH progression and the tape−drum pitch term). CV2 → drum.
    float    effectiveTape = tapeSpeed + g_cv1Value * g_cv1Scale + g_cv1Offset;
    float    effectiveDrum = drumSpeed + g_cv2Value * g_cv2Scale + g_cv2Offset;

    // Segment length is ABSOLUTE (Page-B K2, in samples) — independent of buffer
    // length, so the grain character doesn't shift when the buffer changes. In V8
    // it sets the GRAIN size: how much tape each head scans before grabbing a fresh
    // segment. Short = textured/buzzy; long = smooth/fluid. Clamped to the buffer
    // (can't exceed available memory) and to a sane minimum.
    float segLen = g_segLen;
    if (segLen > (float)bufLen) segLen = (float)bufLen;
    if (segLen < (float)NUM_HEADS) segLen = (float)NUM_HEADS;
    const float arcLen     = segLen;
    const float overlapLen = overlap * arcLen;
    const float halfOverlap = overlapLen * 0.5f;
    const float invOverlapLen = (overlapLen > 0.f) ? 1.0f / overlapLen : 0.f;
    const float norm       = (float)NUM_HEADS * (0.5f + 0.5f * overlap);
    const float invNorm    = 1.0f / norm;

    // (V8: heads no longer use a fixed per-head address offset; they stagger by
    // scan PHASE within the segment, set once at init, so handoffs are spread out.)

    for (int h = 0; h < NUM_HEADS; h++) {
        s_segOrigin[h] = wrapf(s_segOrigin[h], (float)bufLen);
    }
    s_grab      = wrapf(s_grab, (float)bufLen);
    s_writePtrF = wrapf(s_writePtrF, (float)bufLen);
    // Keep the master scan phase bounded to the current segment length. Because
    // every head's phase is DERIVED from this master (+ a fixed per-head offset)
    // rather than accumulated independently, a change in arcLen rescales all heads
    // together and can never break their even stagger.
    s_masterScan = wrapf(s_masterScan, arcLen);

    // Wow/flutter modulation updated once per block (rates are ≤9Hz, so a
    // per-block step at ~172Hz block rate is far above Nyquist for them and
    // sounds identical to per-sample, at a tiny fraction of the sinf cost).
    s_wowPhase     += (0.7f / (float)SAMPLE_RATE) * BLOCK_SIZE;
    s_flutterPhase += (9.0f / (float)SAMPLE_RATE) * BLOCK_SIZE;
    float speedMod  = (1.0f + wow     * sinf(2.0f * M_PI * s_wowPhase))
                    * (1.0f + flutter * sinf(2.0f * M_PI * s_flutterPhase));
    // ── Faithful Tempophon model (V8 — two independent axes) ─────────────────
    // The buffer is the tape (write head fixed at +1.0, unit density, independent).
    // Reading is split into two mechanisms, reproducing how the real machine
    // decouples time from pitch:
    //   PROGRESSION (time): a shared grab playhead s_grab walks through the tape at
    //     the TAPE rate. progRate = tape. Depends on tape ONLY — NOT drum.
    //   PITCH: within each segment, the read scans at the RELATIVE velocity
    //     (tape − drum). scanRate = tape − drum. Each segment is grabbed at the
    //     current s_grab, then scanned at scanRate; at the segment boundary the next
    //     head grabs a fresh (tape-advanced) segment, Hann-crossfading the handoff.
    //   Consequences (validated off-board):
    //     tape == drum   → scanRate 0 → frozen scan → SILENCE (held point)
    //     tape − drum==1 → scans at write density → recorded pitch (unity)
    //     drum > tape    → scanRate < 0 → reverse scan within the grain
    //     tape fixed, drum varied → progression HOLDS, pitch MOVES (the decoupling)
    //     tape == 0      → grab frozen → freeze on a spot
    // Segments repeat/drop to reconcile progRate vs scanRate; the crossfade hides it
    // (this is the mechanism — Carlos's "itty-bitty bits, cross-faded"). wow/flutter
    // modulate the drum (pitch) axis only, as on the real rotor.
    const float progRate = effectiveTape;
    const float scanRate = effectiveTape - effectiveDrum * speedMod;
    const float fbufLen  = (float)bufLen;

    for (int n = 0; n < BLOCK_SIZE; n++) {
        // ── Write head: unit-density recorder, fixed at +1.0 (pre-written tape) ──
        // Records the incoming audio one sample per cell in real time. Tape speed
        // does NOT touch the write — tape is a READ-side term.
        float inMono = mono[n] * inputGain;
        uint32_t wi  = (uint32_t)s_writePtrF;
        s_ring[wi]   = (int16_t)fmaxf(-32768.f, fminf(32767.f,
                           inMono * (1.0f - decay) + (float)s_ring[wi] * decay));
        s_writePtrF += 1.0f;
        if (s_writePtrF >= fbufLen) s_writePtrF -= fbufLen;

        // ── PROGRESSION axis: grab playhead walks the tape at the TAPE rate ──────
        s_grab += progRate;
        if (s_grab >= fbufLen) s_grab -= fbufLen;
        else if (s_grab < 0.f) s_grab += fbufLen;

        // ── PITCH axis: one MASTER scan phase advances at (tape − drum); each head
        //    derives its position from it, evenly staggered — phase-locked, so the
        //    heads can never bunch up regardless of buffer/segment changes. ────────
        s_masterScan += scanRate;
        if (s_masterScan >= arcLen)     s_masterScan -= arcLen;
        else if (s_masterScan < 0.f)    s_masterScan += arcLen;

        const float phaseStep = arcLen / (float)NUM_HEADS;

        float out = 0.0f;
        for (int h = 0; h < NUM_HEADS; h++) {
            // This head's scan phase = master + its fixed stagger offset, wrapped.
            float scanPos = s_masterScan + phaseStep * (float)h;
            if (scanPos >= arcLen) scanPos -= arcLen;   // single fold suffices (offset < arcLen)

            // Segment boundary for THIS head: detect when its derived phase wrapped
            // since last sample (works for both directions). On a wrap it grabs a
            // FRESH segment at the current grab playhead — same repeat/drop
            // reconciliation as before, but the handoff instants stay evenly spread.
            float prev = s_prevSegPhase[h];
            bool wrapped = (scanRate >= 0.f) ? (scanPos < prev)   // forward: phase jumped back
                                             : (scanPos > prev);  // reverse: phase jumped up
            if (wrapped) s_segOrigin[h] = s_grab;
            s_prevSegPhase[h] = scanPos;

            // Hann crossfade at the segment edges to hide the handoff.
            float w;
            if      (scanPos < halfOverlap)            w = hannWindow(scanPos * invOverlapLen);
            else if (scanPos > arcLen - halfOverlap)   w = hannWindow(1.f - (arcLen - scanPos) * invOverlapLen);
            else                                        w = 1.0f;

            // Read address = where this head grabbed (segOrigin) + how far it has
            // scanned within the segment (scanPos). No write-pointer term.
            out += ringRead(s_segOrigin[h] + scanPos, fbufLen) * w;
        }

        mono[n] = fmaxf(-32768.f, fminf(32767.f, out * invNorm * outputGain));
    }
    // Diagnostic: capture the parameter state and a sample raw head-sum so we can
    // see whether the DSP is producing signal that then gets scaled to zero.
    g_dbgOutputGain = outputGain;
    g_dbgInputGain  = inputGain;
    g_dbgNorm       = norm;
    g_dbgTape       = effectiveTape;
    g_dbgDrum       = effectiveDrum;
}

// ── Audio processing — external-buffer model (official AMY custom-dsp pattern) ─
// We do NOT use a render hook anymore. Instead, in loop() we pull the input
// block with amy_get_input_buffer(), process it, and hand it back with
// amy_set_external_input_buffer(). Two oscillators (AUDIO_EXT0 / AUDIO_EXT1,
// set up in setup()) then play it to the L and R outputs respectively. This is
// AMY's intended full-duplex in→out path and avoids the render-hook /
// fixed-point / mixer issues entirely.
//
// The Eltro engine is MONO: we downmix L+R in, run one eltroProcess(), and write
// the same processed mono signal to BOTH channels of the external buffer (so the
// two EXT oscs reproduce identical L=R output — mono duplicated to stereo).

// Diagnostics (read/cleared from loop())
volatile uint32_t g_blockCount = 0;    // audio blocks processed since last report
volatile int32_t  g_inPeak     = 0;    // peak |input| this period
volatile int32_t  g_outPeak    = 0;    // peak |output| this period
volatile int16_t  g_dumpBuf[16];       // snapshot of 16 consecutive OUT samples
volatile uint8_t  g_dumpReady  = 0;    // 0 = arm/fill, 1 = ready to print

// Process one interleaved stereo block in place: read L/R, downmix to mono,
// run the Eltro effect (or pass dry if bypassed), write result to BOTH L and R.
// buf length is AMY_BLOCK_SIZE * AMY_NCHANS, samples are int16 (output_sample_type).
static void eltroProcessBlock(output_sample_type* buf) {
    float mono[BLOCK_SIZE];

    // Downmix interleaved L+R → mono (±32768 float)
    int32_t inPk = 0;
    for (int n = 0; n < BLOCK_SIZE; n++) {
        int16_t l = buf[n * AMY_NCHANS];
        int16_t r = buf[n * AMY_NCHANS + 1];
        mono[n] = ((float)l + (float)r) * 0.5f;
        int32_t a = (int32_t)fabsf(mono[n]);
        if (a > inPk) inPk = a;
    }
    if (inPk > g_inPeak) g_inPeak = inPk;

    // Run the Eltro engine in place. In bypass, eltroProcess records to the tape
    // but leaves `mono` untouched (dry passthrough); otherwise it processes mono.
    eltroProcess(mono);

    // Write processed mono back to BOTH channels (mono → L=R).
    int32_t outPk = 0;
    for (int n = 0; n < BLOCK_SIZE; n++) {
        float v = mono[n];
        if (!isfinite(v)) v = 0.0f;
        if (v >  32767.f) v =  32767.f;
        if (v < -32768.f) v = -32768.f;
        int16_t s = (int16_t)v;
        buf[n * AMY_NCHANS]     = s;
        buf[n * AMY_NCHANS + 1] = s;
        int32_t a = (int32_t)fabsf(v);
        if (a > outPk) outPk = a;
        if (g_dumpReady == 0 && n < 16) g_dumpBuf[n] = s;   // snapshot first 16 out samples
    }
    if (outPk > g_outPeak) g_outPeak = outPk;
    if (g_dumpReady == 0) g_dumpReady = 1;   // arm: loop() will print + re-clear

    g_blockCount++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4.  8ANGLE HELPERS
// ═══════════════════════════════════════════════════════════════════════════════

uint16_t pot_raw[ANGLE8_POTS];
uint16_t pot_prev[ANGLE8_POTS];
uint32_t led_flash_until[ANGLE8_POTS];

// ── 8Angle physical mounting orientation ─────────────────────────────────────
// The M5Stack 8Angle is mounted REVERSED (rotated 180°) so the bypass toggle is
// on the LEFT. That flips the hardware channel order relative to the logical
// left-to-right layout the firmware/labels assume. HW_CH() maps a logical
// channel (0 = leftmost knob) to the physical hardware channel. Set
// ANGLE8_REVERSED to 0 if you ever mount it the original way.
#define ANGLE8_REVERSED 1
// Pot turn DIRECTION — completely independent of mounting. This corrects the
// electrical sense of the wiper (which way is "up"). If clockwise decreases the
// value when it should increase, set this to 1. (Rotating the board in the panel
// does NOT change a pot's CW/CCW sense — only this flag does.)
#define ANGLE8_INVERT_DIR 1
static inline uint8_t HW_CH(uint8_t logical) {
#if ANGLE8_REVERSED
    return (ANGLE8_POTS - 1) - logical;
#else
    return logical;
#endif
}

void readAllPots() {
    for (int ch = 0; ch < ANGLE8_POTS; ch++) {
        uint8_t hw = HW_CH(ch);                 // read leftmost-logical from reversed HW
        Wire.beginTransmission(ANGLE8_ADDR);
        Wire.write((uint8_t)(hw * 2));
        Wire.endTransmission();
        if (Wire.requestFrom(ANGLE8_ADDR, 2) == 2) {
            uint8_t lo = Wire.read();
            uint8_t hi = Wire.read();
            uint16_t v = (uint16_t)((hi << 8) | lo);
#if ANGLE8_INVERT_DIR
            // Pot electrical direction (independent of mounting orientation):
            // set to 1 if clockwise should INCREASE but currently decreases.
            v = (v > 4095) ? 0 : (4095 - v);
#endif
            pot_raw[ch] = v;
        }
    }
}

bool readToggle() {
    Wire.beginTransmission(ANGLE8_ADDR);
    Wire.write(0x20);
    Wire.endTransmission();
    if (Wire.requestFrom(ANGLE8_ADDR, 1) == 1) return (Wire.read() & 1) != 0;
    return false;
}

// ── CV input read: ADS1115 single-ended, single-shot, TWO channels ───────────
// The ADS1115 converts one channel at a time, so we ping-pong: start A0, read
// A0, start A1, read A1, repeat. Two-phase and NON-BLOCKING across uiTask calls
// so we never busy-wait on the bus — each call does at most one short register
// access, preserving the uiTask's single-owner I2C discipline. With the read
// throttled at CV_READ_MS and two channels alternating, each CV updates at
// ~1/(2·CV_READ_MS) ≈ 50 Hz, ample for control voltage. Each channel gets its
// own EMA state and its own ±CV_CLAMP ceiling.
//
// ADS1115 config register (0x01), 16-bit, MSB first:
//   OS=1 (start single-shot) | MUX=100+ch (single-ended AINch vs GND)
//   PGA=001 (±6.144 V FS)     | MODE=1 (single-shot)
//   DR=111 (860 SPS)          | comparator disabled (0x0003)
static bool     s_cvPending   = false;
static uint32_t s_cvStartedMs = 0;
static uint8_t  s_cvCur       = 0;        // which channel is being converted now (0 or 1)
static float    s_cvEma[2]    = {0, 0};
static bool     s_cvEmaInit[2] = {false, false};

static void cvStartConversion(uint8_t adcCh) {
    // MUX for single-ended: 100=A0, 101=A1, 110=A2, 111=A3
    uint16_t mux = 0x4000 | ((uint16_t)(adcCh & 0x03) << 12);
    uint16_t cfg = 0x8000   // OS: begin single conversion
                 | mux      // single-ended channel select
                 | 0x0200   // PGA = ±6.144 V
                 | 0x0100   // MODE = single-shot
                 | 0x00E0   // DR  = 860 SPS
                 | 0x0003;  // comparator disabled
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(0x01);                       // point at config register
    Wire.write((uint8_t)(cfg >> 8));
    Wire.write((uint8_t)(cfg & 0xFF));
    if (Wire.endTransmission() == 0) {
        s_cvPending   = true;
        s_cvStartedMs = millis();
    }
}

static void cvFinishConversion(uint8_t slot) {   // slot: 0 = CV1, 1 = CV2
    Wire.beginTransmission(ADS1115_ADDR);
    Wire.write(0x00);                       // point at conversion register
    if (Wire.endTransmission(false) != 0) { s_cvPending = false; return; }
    if (Wire.requestFrom(ADS1115_ADDR, 2) != 2) { s_cvPending = false; return; }
    uint8_t hi = Wire.read();                      // read MSB then LSB in sequence
    uint8_t lo = Wire.read();                       // (don't rely on arg eval order)
    int16_t raw = (int16_t)(((uint16_t)hi << 8) | lo);  // signed two's-complement
    s_cvPending = false;

    float volts = (float)raw * ADS1115_V_PER_COUNT;   // jack volts (see calibration note)
    float cv    = volts;                              // 1 unit per volt
    if (cv >  CV_CLAMP) cv =  CV_CLAMP;               // ±5.0 reference swing
    if (cv < -CV_CLAMP) cv = -CV_CLAMP;

    if (!s_cvEmaInit[slot]) { s_cvEma[slot] = cv; s_cvEmaInit[slot] = true; }  // seed
    else                    s_cvEma[slot] += CV_EMA_ALPHA * (cv - s_cvEma[slot]);
    if (slot == 0) g_cv1Value = s_cvEma[0];           // CV1 → tape
    else           g_cv2Value = s_cvEma[1];           // CV2 → drum
}

// Called from uiTask on the CV_READ_MS cadence. If a conversion is pending and
// its window has elapsed, collect it and flip to the other channel; otherwise
// start a conversion on the current channel. slot 0 reads A0 (CV1), slot 1 reads
// A1 (CV2).
static void serviceCvInput() {
    const uint8_t adcCh = (s_cvCur == 0) ? ADS1115_CH_CV1 : ADS1115_CH_CV2;
    if (!s_cvPending) {
        cvStartConversion(adcCh);
    } else if (millis() - s_cvStartedMs >= 2) {   // ~1.2 ms conv @860 SPS + margin
        cvFinishConversion(s_cvCur);
        s_cvCur ^= 1;                             // alternate CV1 ↔ CV2
    }
}

void setLed(uint8_t ch, uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
    uint8_t hw = HW_CH(ch);
    Wire.beginTransmission(ANGLE8_ADDR);
    Wire.write((uint8_t)(0x30 + hw * 4));
    Wire.write(r); Wire.write(g); Wire.write(b); Wire.write(bright);
    Wire.endTransmission();
}

void setAllLeds(uint8_t r, uint8_t g, uint8_t b, uint8_t bright) {
    for (int i = 0; i < ANGLE8_POTS; i++) {
        setLed(i, r, g, b, bright);
        delayMicroseconds(200);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5.  PARAMETER MAPPING
// ═══════════════════════════════════════════════════════════════════════════════

// Each mapping law is factored into a *FromNorm(t, ...) core taking a normalized
// t in [0,1], so the physical pot (t = raw/4095) and an incoming MIDI CC
// (t = cc/127) map through EXACTLY the same math — a CC and the equivalent knob
// position give identical parameter values.
float linearFromNorm(float t, float lo, float hi) {
    return lo + t * (hi - lo);
}
float potLinear(uint8_t ch, float lo, float hi) {
    return linearFromNorm(pot_raw[ch] / 4095.f, lo, hi);
}

float bipolarFromNorm(float t, float lo, float centre, float hi) {
    if (fabsf(t - 0.5f) < 0.015f) return centre;
    return (t < 0.5f) ? lo + (t / 0.5f) * (centre - lo)
                      : centre + ((t - 0.5f) / 0.5f) * (hi - centre);
}
float potBipolar(uint8_t ch, float lo, float centre, float hi) {
    return bipolarFromNorm(pot_raw[ch] / 4095.f, lo, centre, hi);
}

// Bipolar mapping with an exponential taper for finer control near centre.
// Symmetric about 0: knob position is taken as a signed x in [−1,+1], reshaped by
// out = sign(x) * |x|^gamma * range. gamma = 1 reproduces potBipolar's linear
// throw; gamma > 1 spends more knob travel on the dense −range_unit..+range_unit
// region around 0. Centre detent preserved. Used for tape and drum (range = the
// knob's full-scale, i.e. g_tapeRange / g_drumRange).
float bipolarTaperFromNorm(float t, float range, float gamma) {
    if (fabsf(t - 0.5f) < 0.015f) return 0.0f;   // centre detent = stopped
    float x = (t - 0.5f) * 2.0f;                 // signed position, −1..+1
    float mag = powf(fabsf(x), gamma);           // reshape magnitude only
    return copysignf(mag, x) * range;            // restore sign, scale to range
}
float potBipolarTaper(uint8_t ch, float range, float gamma) {
    return bipolarTaperFromNorm(pot_raw[ch] / 4095.f, range, gamma);
}

uint32_t bufLenFromNorm(float t) {
    float logMin = log2f((float)RING_BUF_MIN);
    float logMax = log2f((float)RING_BUF_MAX);
    uint32_t l   = (uint32_t)(powf(2.0f, logMin + t * (logMax - logMin)));
    l = (l / NUM_HEADS) * NUM_HEADS;
    return (l < RING_BUF_MIN) ? RING_BUF_MIN : (l > RING_BUF_MAX ? RING_BUF_MAX : l);
}
uint32_t potBufLen(uint8_t ch) {
    return bufLenFromNorm(pot_raw[ch] / 4095.f);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Page-B persistence + catch/pickup
//   Page-B settings (seg divisor, tape/drum range, CV scale/offset) are saved to
//   NVS flash and reloaded on boot, so they survive power cycles. On boot (and
//   whenever Page B is entered) each Page-B pot is "waiting": it does NOT change
//   its parameter until physically moved past PAGEB_CATCH_THRESH from the reading
//   it had at that moment. Once moved it is "caught" and live; every change is
//   re-saved (debounced). This lets the saved value persist even though the pot
//   sits at an arbitrary physical position after power-up.
// ═══════════════════════════════════════════════════════════════════════════════
static Preferences s_prefs;

#define PAGEB_POTS        8        // logical channels 0..7 are the Page-B params (K7 = MIDI channel)
#define PAGEB_CATCH_THRESH 40      // raw ADC move (of 4095) needed to "catch" a pot

static bool     s_pbCaught[PAGEB_POTS]   = { false, false, false, false, false, false, false, false };
static uint16_t s_pbRefRaw[PAGEB_POTS]   = { 0 };     // reading when waiting began

// V9 Page-A MIDI catch-takeover: when a CC sets a Page-A parameter, that knob
// becomes "MIDI-owned" — the physical pot is ignored for that param until it is
// moved past MIDI_CATCH_THRESH from where it sat when MIDI took over. Per-knob,
// so you can automate tape by MIDI while tweaking overlap by hand. Cleared the
// moment the pot reclaims the parameter.
static bool     s_midiOwned[ANGLE8_POTS]  = { false };
static uint16_t s_midiRefRaw[ANGLE8_POTS] = { 0 };

// V12 Page-A pickup (catch) on page RE-ENTRY. Mirrors Page B's mechanism. When
// Page A is (re)entered, all eight knobs are armed "waiting" at their current
// reading; a knob does not drive its Page-A parameter until moved past
// PAGEA_PICKUP_THRESH, so values held during a Page-B visit (or set by MIDI) are
// preserved until the hand deliberately grabs a knob. s_pageAArmed gates this:
// it is FALSE at boot (Page A is live at power-on — knobs take their physical
// position immediately) and becomes TRUE only when Page A is re-entered via the
// page toggle. Independent of the MIDI catch-takeover (s_midiOwned) above.
#define PAGEA_PICKUP_THRESH 40      // raw ADC move (of 4095) needed to grab a knob
static bool     s_pageAArmed              = false;   // false at boot → live; true after re-entry
static bool     s_paPicked[ANGLE8_POTS]   = { false };
static uint16_t s_paRefRaw[ANGLE8_POTS]   = { 0 };

static bool     s_pbDirty                = false;     // unsaved changes pending
static uint32_t s_pbDirtySince           = 0;         // millis of first pending change

// Load persisted Page-B values (or defaults if never saved).
static void loadPageBSettings() {
    s_prefs.begin("eltro", true);   // read-only
    // CV1 keeps the original "cvsc"/"cvof" keys so any previously-saved single-CV
    // values migrate straight into CV1 on first boot after the two-CV change.
    // CV2 gets new keys and starts at defaults until you move its pots.
    g_cv1Scale  = s_prefs.getFloat("cvsc",  g_cv1Scale);
    g_cv1Offset = s_prefs.getFloat("cvof",  g_cv1Offset);
    g_cv2Scale  = s_prefs.getFloat("cv2sc", g_cv2Scale);
    g_cv2Offset = s_prefs.getFloat("cv2of", g_cv2Offset);
    g_segLen    = s_prefs.getFloat("seg",  g_segLen);
    g_tapeRange = s_prefs.getFloat("trng", g_tapeRange);
    g_drumRange = s_prefs.getFloat("drng", g_drumRange);
    g_midiChannel = (uint8_t)s_prefs.getUChar("mch", g_midiChannel);
    if (g_midiChannel < 1 || g_midiChannel > 16) g_midiChannel = 1;
    g_speedGamma = s_prefs.getFloat("sgam", g_speedGamma);
    s_prefs.end();
}

// Persist current Page-B values to flash.
static void savePageBSettings() {
    s_prefs.begin("eltro", false);  // read-write
    s_prefs.putFloat("cvsc",  g_cv1Scale);
    s_prefs.putFloat("cvof",  g_cv1Offset);
    s_prefs.putFloat("cv2sc", g_cv2Scale);
    s_prefs.putFloat("cv2of", g_cv2Offset);
    s_prefs.putFloat("seg",  g_segLen);
    s_prefs.putFloat("trng", g_tapeRange);
    s_prefs.putFloat("drng", g_drumRange);
    s_prefs.putUChar("mch", g_midiChannel);
    s_prefs.putFloat("sgam", g_speedGamma);
    s_prefs.end();
    s_pbDirty = false;
}

// Re-arm catch/pickup: all Page-B pots become "waiting" at their current reading.
// Called on boot and whenever Page B is (re)entered.
static void rearmPageBCatch() {
    for (int i = 0; i < PAGEB_POTS; i++) {
        s_pbCaught[i] = false;
        s_pbRefRaw[i] = pot_raw[i];
    }
}

// V12: re-arm Page-A pickup — all Page-A pots become "waiting" at their current
// reading, so re-entering Page A won't snap parameters to the physical pots.
// Sets s_pageAArmed so the pickup gate in potDrivesPageA() takes effect. NOT
// called on boot (Page A is live at power-on); called only on Page-A re-entry.
static void rearmPageACatch() {
    s_pageAArmed = true;
    for (int i = 0; i < ANGLE8_POTS; i++) {
        s_paPicked[i] = false;
        s_paRefRaw[i] = pot_raw[i];
    }
}

// Returns true if Page-A pot `ch` is allowed to drive its parameter.
//  1) PICKUP GATE (V12): after a Page-A re-entry (s_pageAArmed), a knob is held
//     until physically moved past PAGEA_PICKUP_THRESH from its reading on entry —
//     then it's "picked" and drives normally (and any MIDI ownership is dropped,
//     since the hand has grabbed it). At boot s_pageAArmed is false, so Page A is
//     live immediately.
//  2) MIDI CATCH: if the knob is MIDI-owned (a CC set it), the physical pot is
//     ignored until moved past MIDI_CATCH_THRESH from where it sat when MIDI took
//     over — then the hand reclaims it and ownership clears.
static bool potDrivesPageA(int ch) {
    // (1) pickup gate on page re-entry
    if (s_pageAArmed && !s_paPicked[ch]) {
        int d = (int)pot_raw[ch] - (int)s_paRefRaw[ch];
        if (d < 0) d = -d;
        if (d > PAGEA_PICKUP_THRESH) {
            s_paPicked[ch]  = true;    // hand grabbed this knob
            s_midiOwned[ch] = false;   // grabbing overrides any MIDI ownership
            return true;
        }
        return false;                  // still waiting → hold the current value
    }
    // (2) MIDI catch-takeover (unchanged)
    if (!s_midiOwned[ch]) return true;
    int delta = (int)pot_raw[ch] - (int)s_midiRefRaw[ch];
    if (delta < 0) delta = -delta;
    if (delta > MIDI_CATCH_THRESH) {
        s_midiOwned[ch] = false;   // hand takes back control
        return true;
    }
    return false;                  // MIDI keeps the value
}

void applyPotsPageA() {
    // Faithful Tempophon mapping: TAPE = tape transport speed, DRUM = drum/head
    // rotor speed, each bipolar (−range…0…+range; reverse…stopped…forward). What
    // you hear is the relative read-through speed (tape − drum):
    //   tape == drum → SILENCE (no relative motion)
    //   tape − drum == 1 → plays at recorded pitch (unity)
    //   drum > tape → reverse;  drum opposing tape → pitch up.
    // Ranges set on Page B (K5 tape, K6 drum, 1.0–5.0).
    // V9: each knob only drives its param if not currently MIDI-owned (per-knob
    // catch takeover — see potDrivesPageA).
    if (potDrivesPageA(0)) g_tapeSpeed  = potBipolarTaper(0, g_tapeRange, g_speedGamma);
    if (potDrivesPageA(1)) g_drumSpeed  = potBipolarTaper(1, g_drumRange, g_speedGamma);
    if (potDrivesPageA(2)) g_overlap    = potLinear (2,  0.10f, 0.95f);   // V8: floor raised from 0.01 — below ~0.10 the granular seams click
    if (potDrivesPageA(3)) g_wow        = potLinear (3,  0.0f,  0.05f);
    if (potDrivesPageA(4)) g_inputGain  = potLinear (4,  0.0f,  4.0f);   // was 2.0; raised for quiet AMYboard input
    if (potDrivesPageA(5)) g_outputGain = potLinear (5,  0.0f,  1.5f);
    if (potDrivesPageA(6)) g_decay      = potLinear (6,  0.0f,  0.9999f);
    if (potDrivesPageA(7)) g_bufLen     = potBufLen (7);
}

// Returns true if Page-B pot `ch` is allowed to drive its parameter: either it
// is already caught, or it has just moved past the catch threshold (in which case
// we latch it caught). Until caught, the persisted value is left untouched.
static bool pageBCaught(int ch) {
    if (s_pbCaught[ch]) return true;
    int delta = (int)pot_raw[ch] - (int)s_pbRefRaw[ch];
    if (delta < 0) delta = -delta;
    if (delta > PAGEB_CATCH_THRESH) {
        s_pbCaught[ch] = true;
        return true;
    }
    return false;
}

void applyPotsPageB() {
    // Each parameter updates ONLY once its pot is "caught" (physically moved since
    // Page B was entered / since boot). This preserves the persisted values until
    // you deliberately grab a knob. Any change marks the settings dirty so they
    // get saved (debounced) by the uiTask.
    // New Page-B layout (V8+): K0/K1 = CV1 (→tape) scale/offset, K2/K3 = CV2
    // (→drum) scale/offset, K4 = segment length, K5 = tape range, K6 = drum range.
    if (pageBCaught(0)) { float v = potLinear (0,  0.0f,  4.0f);        if (v != g_cv1Scale)  { g_cv1Scale  = v; s_pbDirty = true; } }
    if (pageBCaught(1)) { float v = potBipolar(1, -2.0f, 0.0f, 2.0f);   if (v != g_cv1Offset) { g_cv1Offset = v; s_pbDirty = true; } }
    if (pageBCaught(2)) { float v = potLinear (2,  0.0f,  4.0f);        if (v != g_cv2Scale)  { g_cv2Scale  = v; s_pbDirty = true; } }
    if (pageBCaught(3)) { float v = potBipolar(3, -2.0f, 0.0f, 2.0f);   if (v != g_cv2Offset) { g_cv2Offset = v; s_pbDirty = true; } }
    if (pageBCaught(4)) { float v = potLinear (4, (float)SEG_MIN, (float)SEG_MAX); if (v != g_segLen) { g_segLen = v; s_pbDirty = true; } }
    if (pageBCaught(5)) { float v = potLinear (5,  1.0f,  5.0f);        if (v != g_tapeRange) { g_tapeRange = v; s_pbDirty = true; } }
    if (pageBCaught(6)) { float v = potLinear (6,  1.0f,  5.0f);        if (v != g_drumRange) { g_drumRange = v; s_pbDirty = true; } }
    if (pageBCaught(7)) { uint8_t v = (uint8_t)(potLinear(7, 1.0f, 16.999f)); if (v < 1) v = 1; if (v > 16) v = 16; if (v != g_midiChannel) { g_midiChannel = v; s_pbDirty = true; } }
    if (s_pbDirty && s_pbDirtySince == 0) s_pbDirtySince = millis();
}

void applyPots() {
    if (g_pageB) applyPotsPageB();
    else         applyPotsPageA();
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5b.  MIDI RX (V9) — byte-at-a-time state machine + fixed CC map for Page A
//   Replaces the old fixed 3-byte reader, which mis-framed on RUNNING STATUS (a
//   controller streaming CCs omits the repeated status byte). This parser tracks
//   running status, ignores real-time bytes (0xF8–0xFF) wherever they appear, and
//   dispatches only Control Change; other channel-voice messages are parsed (so
//   framing stays correct) but not acted on. No clock/sync — CC only.
// ═══════════════════════════════════════════════════════════════════════════════

// Take a Page-A parameter under MIDI control: set the value from a 0..1 normalized
// CC, mark the knob MIDI-owned, and latch the current pot position so the physical
// knob stays ignored until moved past MIDI_CATCH_THRESH.
static void midiSetPageA(uint8_t knob, float value) {
    switch (knob) {
        case 0: g_tapeSpeed  = value; break;   // caller passes already-mapped value
        case 1: g_drumSpeed  = value; break;
        case 2: g_overlap    = value; break;
        case 3: g_wow        = value; break;
        case 4: g_inputGain  = value; break;
        case 5: g_outputGain = value; break;
        case 6: g_decay      = value; break;
        default: return;
    }
    s_midiOwned[knob]  = true;
    s_midiRefRaw[knob] = pot_raw[knob];   // reclaim only after the pot moves from here
}

static void midiHandleCC(uint8_t cc, uint8_t val) {
    const float t = val / 127.0f;         // normalize like a pot's raw/4095
    switch (cc) {
        case MIDI_CC_TAPE:    midiSetPageA(0, bipolarTaperFromNorm(t, g_tapeRange, g_speedGamma)); break;
        case MIDI_CC_DRUM:    midiSetPageA(1, bipolarTaperFromNorm(t, g_drumRange, g_speedGamma)); break;
        case MIDI_CC_OVERLAP: midiSetPageA(2, linearFromNorm(t, 0.10f, 0.95f));   break;
        case MIDI_CC_WOW:     midiSetPageA(3, linearFromNorm(t, 0.0f, 0.05f));    break;
        case MIDI_CC_INGAIN:  midiSetPageA(4, linearFromNorm(t, 0.0f, 4.0f));     break;
        case MIDI_CC_OUTGAIN: midiSetPageA(5, linearFromNorm(t, 0.0f, 1.5f));     break;
        case MIDI_CC_DECAY:   midiSetPageA(6, linearFromNorm(t, 0.0f, 0.9999f));  break;
        case MIDI_CC_BUFLEN:  // buffer length: knob 7, log law; owns knob 7 directly
            g_bufLen = bufLenFromNorm(t);
            s_midiOwned[7] = true; s_midiRefRaw[7] = pot_raw[7];
            break;
        case MIDI_CC_PAGE: {  // preserve existing page-toggle behavior
            bool newPage = (val >= 64);
            if (newPage != (bool)g_pageB) {
                bool wasB = g_pageB;
                g_pageB = newPage;
                if (g_pageB && !wasB) rearmPageBCatch();
                if (!g_pageB && wasB) rearmPageACatch();   // entering A: pickup, don't snap
                if (!g_pageB && wasB && s_pbDirty) savePageBSettings();
                applyPots();
                updateLeds(g_bypass, g_pageB);
                Serial.printf("Page (MIDI CC65): %s\n", g_pageB ? "B" : "A");
            }
            break;
        }
        default: break;   // unmapped CC ignored
    }
}

// Byte-at-a-time parser state.
static uint8_t s_midiRunningStatus = 0;     // last channel-voice status (0 = none)
static uint8_t s_midiData[2];               // data-byte accumulator
static uint8_t s_midiDataIdx  = 0;          // how many data bytes gathered
static uint8_t s_midiExpected = 0;          // data bytes expected for current status

// data-byte count for a channel-voice status high-nibble
static uint8_t midiDataLen(uint8_t status) {
    switch (status & 0xF0) {
        case 0xC0: case 0xD0: return 1;     // program change, channel pressure
        default:              return 2;     // note off/on, poly AT, CC, pitch bend
    }
}

static void midiFeedByte(uint8_t b) {
    if (b >= 0xF8) return;                   // real-time (clock etc.): ignore, don't disturb state
    if (b >= 0x80) {                         // status byte
        if (b < 0xF0) {                      // channel-voice: becomes running status
            s_midiRunningStatus = b;
            s_midiExpected = midiDataLen(b);
            s_midiDataIdx  = 0;
        } else {                             // system-common: clears running status
            s_midiRunningStatus = 0;
            s_midiDataIdx = 0;
        }
        return;
    }
    // data byte
    if (s_midiRunningStatus == 0) return;    // no status yet; drop
    s_midiData[s_midiDataIdx++] = b;
    if (s_midiDataIdx >= s_midiExpected) {
        s_midiDataIdx = 0;                   // ready for the next (running-status) message
        uint8_t hi = s_midiRunningStatus & 0xF0;
        uint8_t ch = (s_midiRunningStatus & 0x0F) + 1;   // 1..16
        if (ch == g_midiChannel && hi == 0xB0) {         // Control Change on our channel
            midiHandleCC(s_midiData[0], s_midiData[1]);
        }
        // other message types: framing consumed, no action
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5c.  MIDI TX (V11) — knob → CC out, physical moves only, never echo MIDI.
//   Emits the eight Page-A CCs (102–109) when a knob is PHYSICALLY moved, so a
//   DAW/sequencer can record and play back knob motion. Two guards stop an RX/TX
//   feedback loop:
//     (1) midiTxPageA() is called ONLY inside uiTask's any_changed / POT_DEADBAND
//         path — i.e. only on real physical pot motion, and only on Page A.
//     (2) it skips any knob currently MIDI-owned (s_midiOwned[ch]) — a value that
//         an incoming CC just set is never echoed back out. TX resumes for that
//         knob the instant the hand reclaims it (potDrivesPageA clears ownership).
//   Deadband = 7-bit re-quantization: a knob emits only when its 0..127 value
//   actually changes, so a caught pot resting under a finger never spews CCs.
//   Value sent = raw knob POSITION (raw>>5). RX applies the bipolar taper / log
//   law on the way back in, identically on both ends, so a DAW round-trips a knob
//   to the same sound. (Sending the tapered value would be lossy at the centre-
//   detent flat-spot, where several CC codes collapse to one param value.)
// ═══════════════════════════════════════════════════════════════════════════════

// CC number for each Page-A knob 0..7 — same map as RX (midiHandleCC).
static const uint8_t kKnobCC[ANGLE8_POTS] = {
    MIDI_CC_TAPE, MIDI_CC_DRUM, MIDI_CC_OVERLAP, MIDI_CC_WOW,
    MIDI_CC_INGAIN, MIDI_CC_OUTGAIN, MIDI_CC_DECAY, MIDI_CC_BUFLEN
};

static uint8_t s_lastTxCC[ANGLE8_POTS];   // last 0..127 value sent per knob
static bool    s_txPrimed = false;        // first pass captures baseline, sends nothing

static inline void midiSendCC(uint8_t cc, uint8_t val) {
    Serial1.write((uint8_t)(0xB0 | ((g_midiChannel - 1) & 0x0F)));  // CC status on our channel
    Serial1.write((uint8_t)(cc & 0x7F));
    Serial1.write((uint8_t)(val & 0x7F));
}

// Call from uiTask AFTER applyPots(), only when pots physically changed and only
// on Page A (guard (1) is the caller's if-block; guard (2) is the s_midiOwned skip).
static void midiTxPageA() {
    for (int ch = 0; ch < ANGLE8_POTS; ch++) {
        if (s_midiOwned[ch]) continue;             // (2) never echo a MIDI-driven knob
        uint8_t v = (uint8_t)(pot_raw[ch] >> 5);   // 0..4095 -> 0..127 (knob position)
        if (!s_txPrimed || v != s_lastTxCC[ch]) {  // deadband: only on real 7-bit change
            s_lastTxCC[ch] = v;
            if (s_txPrimed) midiSendCC(kKnobCC[ch], v);
        }
    }
    s_txPrimed = true;   // baseline captured; subsequent changes transmit
}

// Single source of truth for the Page-A pitch LED color, so the two places that
// drive the LEDs (updateLeds + the per-LED flash-restore loop in uiTask) can't
// drift apart. Reads the EFFECTIVE pitch axis (g_dbgTape/g_dbgDrum = knob + CV +
// MIDI), so the LEDs reflect what's really happening, not just knob position.
//   readthru = tape − drum ;  shift = readthru − 1 (0 = recorded/unity pitch)
//   silence (tape≈drum) → dim red ; unity → green ; up → blue ; down → amber.
static void ledColorForState(uint8_t& r, uint8_t& g, uint8_t& b) {
    float readthru = g_dbgTape - g_dbgDrum;
    float shift    = readthru - READTHRU_UNITY;
    if      (fabsf(readthru) < 0.05f) { r=120; g=0;   b=0;   }  // silence (tape==drum)
    else if (fabsf(shift) < 0.05f)    { r=0;   g=100; b=0;   }  // unity pitch (green)
    else if (shift > 0.0f)            { r=0;   g=0;   b=200; }  // faster → up (blue)
    else                              { r=200; g=80;  b=0;   }  // slower → down (amber)
}

void updateLeds(bool bypass, bool pageB) {
    if (bypass) { setAllLeds(200, 0, 0, 60); return; }
    if (pageB)  { setAllLeds(0, 200, 0, 50); return; }
    uint8_t r, g, b;
    ledColorForState(r, g, b);
    setAllLeds(r, g, b, 50);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6.  DISPLAY  (SH1106 128×64, direct I2C driver — no library)
// ═══════════════════════════════════════════════════════════════════════════════
//
// Hand-rolled SH1106 driver using the same explicit Wire transaction pattern as
// the 8Angle (which works flawlessly on this shared bus). Every endTransmission()
// return code is checked, so a bus hiccup yields a skipped frame, never a hang.
// No third-party display library is involved.
//
// The SH1106 controller is 132 columns wide internally but the panel shows the
// middle 128, so all column addressing is offset by +2 (SH1106_COL_OFFSET).
// We keep a 128×64 monochrome framebuffer (1 bit/pixel, 1024 bytes) organised as
// 8 pages of 128 bytes; each byte is a vertical strip of 8 pixels (LSB = top).

#define OLED_ADDR        0x3C
#define OLED_W           128
#define OLED_H           64
#define OLED_PAGES       (OLED_H / 8)          // 8
#define SH1106_COL_OFFSET 2

static uint8_t s_fb[OLED_W * OLED_PAGES];      // 1024-byte framebuffer

static const char* KnobNamesA[ANGLE8_POTS] = {
    "TAPE", "DRUM", "OVLP", "W/F", "IN", "OUT", "DCAY", "BLEN"
};
static const char* KnobNamesB[ANGLE8_POTS] = {
    "C1SC", "C1OF", "C2SC", "C2OF", "SEG", "TRNG", "DRNG", "MCH"
};

// ── Low-level I2C: send a command byte (control byte 0x00) ───────────────────
// Returns true on success. Never blocks beyond a single Wire transaction.
static bool oledCmd(uint8_t c) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write((uint8_t)0x00);   // Co=0, D/C#=0 → command stream
    Wire.write(c);
    return Wire.endTransmission() == 0;
}

// ── 5×7 font ─────────────────────────────────────────────────────────────────
// Compact ASCII font, 5 columns per glyph (+1 spacing). Covers 0x20–0x7A.
// Each glyph is 5 bytes, each byte a vertical column (LSB = top row).
#include "eltro_font5x7.h"   // generated table: font5x7[(c-0x20)*5 + col]

// Draw a single character at (x,y) top-left. Sets bits in the framebuffer.
static void oledChar(int x, int y, char ch) {
    if (ch < 0x20 || ch > 0x7A) ch = '?';
    const uint8_t* g = &font5x7[(ch - 0x20) * 5];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        int px = x + col;
        if (px < 0 || px >= OLED_W) continue;
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                int py = y + row;
                if (py < 0 || py >= OLED_H) continue;
                s_fb[px + (py / 8) * OLED_W] |= (1 << (py & 7));
            }
        }
    }
}

// Draw a string. Returns the x position after the last glyph.
static int oledStr(int x, int y, const char* s) {
    while (*s) {
        oledChar(x, y, *s++);
        x += 6;   // 5px glyph + 1px space
    }
    return x;
}

static int oledStrWidth(const char* s) {
    int n = 0; while (s[n]) n++;
    return n * 6 - (n > 0 ? 1 : 0);
}

// Draw a glyph at 2x scale (10x14) for prominent, readable values.
static void oledChar2x(int x, int y, char ch) {
    if (ch < 0x20 || ch > 0x7A) ch = '?';
    const uint8_t* g = &font5x7[(ch - 0x20) * 5];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                for (int dx = 0; dx < 2; dx++)
                    for (int dy = 0; dy < 2; dy++) {
                        int px = x + col * 2 + dx;
                        int py = y + row * 2 + dy;
                        if (px < 0 || px >= OLED_W || py < 0 || py >= OLED_H) continue;
                        s_fb[px + (py / 8) * OLED_W] |= (1 << (py & 7));
                    }
            }
        }
    }
}
static int oledStr2x(int x, int y, const char* s) {
    while (*s) { oledChar2x(x, y, *s++); x += 12; }   // 10px glyph + 2px space
    return x;
}
static int oledStr2xWidth(const char* s) {
    int n = 0; while (s[n]) n++;
    return n * 12 - (n > 0 ? 2 : 0);
}

// Draw a glyph at ~1.5x scale (8x11). A bitmap font can only scale by whole
// pixels, so "1.5x" is done by duplicating a fixed subset of columns/rows in a
// stair-step: the 5 source columns map to 8 (cols 1 and 3 doubled) and the 7
// source rows map to 11 (rows 1, 3, 5 doubled). Result is ~25% smaller than the
// 2x (10x14) glyph but still clearly larger than the 5x7 base, and stays crisp
// because the duplication pattern is fixed (no interpolation).
//   column source indices → 8 dest cols: 0,1,1,2,3,3,4  (+ one trailing per below)
// We build the destination mapping explicitly for clarity.
static const uint8_t S15_COL[8] = {0,1,1,2,3,3,4,4};  // 5 src cols → 8 dest cols
static const uint8_t S15_ROW[11] = {0,1,1,2,3,3,4,5,5,6,6}; // 7 src rows → 11 dest rows
static void oledChar1p5x(int x, int y, char ch) {
    if (ch < 0x20 || ch > 0x7A) ch = '?';
    const uint8_t* g = &font5x7[(ch - 0x20) * 5];
    for (int dc = 0; dc < 8; dc++) {
        uint8_t bits = g[S15_COL[dc]];
        int px = x + dc;
        if (px < 0 || px >= OLED_W) continue;
        for (int dr = 0; dr < 11; dr++) {
            if (bits & (1 << S15_ROW[dr])) {
                int py = y + dr;
                if (py < 0 || py >= OLED_H) continue;
                s_fb[px + (py / 8) * OLED_W] |= (1 << (py & 7));
            }
        }
    }
}
static int oledStr1p5x(int x, int y, const char* s) {
    while (*s) { oledChar1p5x(x, y, *s++); x += 9; }   // 8px glyph + 1px space
    return x;
}
static int oledStr1p5xWidth(const char* s) {
    int n = 0; while (s[n]) n++;
    return n * 9 - (n > 0 ? 1 : 0);
}

// ── Framebuffer primitives ───────────────────────────────────────────────────
static inline void oledPixel(int x, int y, bool on) {
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    uint8_t* p = &s_fb[x + (y / 8) * OLED_W];
    uint8_t  m = 1 << (y & 7);
    if (on) *p |= m; else *p &= ~m;
}

static void oledClear() { memset(s_fb, 0, sizeof(s_fb)); }

static void oledHLine(int x, int y, int w) {
    for (int i = 0; i < w; i++) oledPixel(x + i, y, true);
}
static void oledVLine(int x, int y, int h) {
    for (int i = 0; i < h; i++) oledPixel(x, y + i, true);
}
static void oledFrame(int x, int y, int w, int h) {
    oledHLine(x, y, w); oledHLine(x, y + h - 1, w);
    oledVLine(x, y, h); oledVLine(x + w - 1, y, h);
}
static void oledBox(int x, int y, int w, int h) {
    for (int j = 0; j < h; j++) oledHLine(x, y + j, w);
}
// XOR a vertical line (for centre ticks that stay visible over fills)
static void oledVLineXor(int x, int y, int h) {
    for (int i = 0; i < h; i++) {
        int py = y + i;
        if (x < 0 || x >= OLED_W || py < 0 || py >= OLED_H) continue;
        s_fb[x + (py / 8) * OLED_W] ^= (1 << (py & 7));
    }
}

// ── Flush framebuffer to the panel, page by page ─────────────────────────────
// Each page is 128 bytes. We send the page/column address commands, then stream
// the 128 data bytes. Every transaction's return code is checked; on any error
// we abort this frame (it'll simply be retried next DISPLAY_MS tick).
static void oledFlush() {
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        if (!oledCmd(0xB0 | page)) return;                       // set page
        if (!oledCmd(0x00 | (SH1106_COL_OFFSET & 0x0F))) return; // lower col
        if (!oledCmd(0x10 | (SH1106_COL_OFFSET >> 4)))   return; // higher col

        // Stream 128 data bytes for this page. Arduino Wire's default 32-byte
        // TX buffer means we chunk: control byte 0x40 + up to 16 data bytes
        // per transaction (1 control + 16 data = 17 ≤ 32).
        const uint8_t* src = &s_fb[page * OLED_W];
        int col = 0;
        while (col < OLED_W) {
            Wire.beginTransmission(OLED_ADDR);
            Wire.write((uint8_t)0x40);   // Co=0, D/C#=1 → data stream
            int chunk = OLED_W - col;
            if (chunk > 16) chunk = 16;
            for (int i = 0; i < chunk; i++) Wire.write(src[col + i]);
            if (Wire.endTransmission() != 0) return;   // bus error → skip frame
            col += chunk;
        }
    }
}

// ── Init sequence (standard SH1106 power-on) ─────────────────────────────────
static bool oledInit() {
    delay(50);
    static const uint8_t init_seq[] = {
        0xAE,             // display off
        0xD5, 0x80,       // clock divide
        0xA8, 0x3F,       // multiplex = 63
        0xD3, 0x00,       // display offset = 0
        0x40,             // start line = 0
        0xAD, 0x8B,       // charge pump on (SH1106)
        0xA1,             // segment remap (mirror X)
        0xC8,             // COM scan direction (flip Y)
        0xDA, 0x12,       // COM pins config
        0x81, 0x80,       // contrast
        0xD9, 0x22,       // pre-charge
        0xDB, 0x40,       // VCOM deselect
        0xA4,             // resume to RAM content
        0xA6,             // normal (non-inverted)
        0xAF              // display on
    };
    bool ok = true;
    for (uint8_t i = 0; i < sizeof(init_seq); i++) {
        Serial.printf("[oled] cmd %d/0x%02X...\n", i, init_seq[i]);
        ok &= oledCmd(init_seq[i]);
        delay(2);   // small gap between commands — stabilizes marginal I2C
    }
    delay(50);
    return ok;
}

// Horizontal bar gauge with filled region from centre to value.
// XOR centre tick stays visible inside or outside the fill.
void drawGauge(int x, int y, int w, int h,
               float val, float lo, float centre, float hi) {
    oledFrame(x, y, w, h);
    float normC = (centre - lo) / (hi - lo);
    float normV = fmaxf(0.f, fminf(1.f, (val - lo) / (hi - lo)));
    int   cx    = x + (int)(normC * (w - 2)) + 1;
    int   vx    = x + (int)(normV * (w - 2)) + 1;
    if (vx > cx) oledBox(cx, y + 1, vx - cx, h - 2);
    else if (vx < cx) oledBox(vx, y + 1, cx - vx, h - 2);
    oledVLineXor(cx, y, h);
}

static void drawDisplay() {
    bool  pageB = g_pageB;

    oledClear();

    // ── Row 0: Title + status badge ──────────────────────────────────────────
    oledStr(0, 1, "ELTRO MK3");
    const char* badge = g_bypass ? "BYP" : (pageB ? "PG B" : "PG A");
    oledStr(127 - oledStrWidth(badge), 1, badge);
    oledHLine(0, 9, 128);

    if (!pageB) {
        // ════ Page A — TAPE and DRUM gauges + all 6 secondary params below ════
        // Each speed row: small label · gauge · big 2x value. Centre tick = 0 =
        // stopped; bar filling LEFT of centre = reverse/negative. Below the rule,
        // all six secondary parameters are shown permanently, two per line.

        // ── TAPE row ─────────────────────────────────────────────────────────
        oledStr(0, 14, "TAP");
        if (s_midiOwned[0])    oledStr(18, 14, ">");   // MIDI-owned (arrow marker, not caption letter)
        if (g_cv1Scale >= 0.005f || fabsf(g_cv1Offset) >= 0.005f) oledStr(24, 14, "c");   // CV1 armed (scale or offset rounds to >0.00)
        {
            // Big number = EFFECTIVE speed (knob + CV), matching the serial diag;
            // the gauge below stays on the KNOB position (stable, in-range — the
            // effective value can exceed the knob range once CV adds on top).
            // In bypass g_dbgTape is stale, so fall back to the knob value there.
            float shown = g_bypass ? g_tapeSpeed : g_dbgTape;
            char v[8]; snprintf(v, sizeof(v), "%+.2f", shown);
            int vw = oledStr1p5xWidth(v);
            oledStr1p5x(128 - vw, 11, v);
            int gx = 30, gright = 128 - vw - 4;
            if (gright > gx + 10) drawGauge(gx, 14, gright - gx, 6, g_tapeSpeed, -g_tapeRange, 0.f, g_tapeRange);
        }

        // ── DRUM row ─────────────────────────────────────────────────────────
        oledStr(0, 29, "DRM");
        if (s_midiOwned[1])    oledStr(18, 29, ">");   // MIDI-owned (arrow marker, not caption letter)
        if (g_cv2Scale >= 0.005f || fabsf(g_cv2Offset) >= 0.005f) oledStr(24, 29, "c");   // CV2 armed (scale or offset rounds to >0.00)
        {
            float shown = g_bypass ? g_drumSpeed : g_dbgDrum;
            char v[8]; snprintf(v, sizeof(v), "%+.2f", shown);
            int vw = oledStr1p5xWidth(v);
            oledStr1p5x(128 - vw, 26, v);
            int gx = 30, gright = 128 - vw - 4;
            if (gright > gx + 10) drawGauge(gx, 29, gright - gx, 6, g_drumSpeed, -g_drumRange, 0.f, g_drumRange);
        }

        oledHLine(0, 39, 128);

        // ── Secondary params: all six, two per line. A leading '>' on a label
        //    marks that knob as currently MIDI-owned (pot reclaims it when moved).
        char line[28];
        snprintf(line, sizeof(line), "%cOVL %.2f %cWOW %.2f",
                 s_midiOwned[2] ? '>' : ' ', g_overlap, s_midiOwned[3] ? '>' : ' ', g_wow);
        oledStr(0, 41, line);
        snprintf(line, sizeof(line), "%cIN  %.2f %cOUT %.2f",
                 s_midiOwned[4] ? '>' : ' ', g_inputGain, s_midiOwned[5] ? '>' : ' ', g_outputGain);
        oledStr(0, 49, line);
        snprintf(line, sizeof(line), "%cDCY %.2f %cBUF %.0f",
                 s_midiOwned[6] ? '>' : ' ', g_decay, s_midiOwned[7] ? '>' : ' ', g_bufLen / 44100.f * 1000.f);
        oledStr(0, 57, line);

    } else {
        // ── Page B: config (CV1→tape, CV2→drum, seg, speed ranges) ───────────
        char l[28];
        snprintf(l, sizeof(l), "CV1>TAP x%.2f %+.2f", g_cv1Scale, g_cv1Offset);
        oledStr(0, 12, l);
        snprintf(l, sizeof(l), "CV2>DRM x%.2f %+.2f", g_cv2Scale, g_cv2Offset);
        oledStr(0, 21, l);
        oledHLine(0, 30, 128);

        // Segment: absolute length in samples + derived ms (independent of buffer).
        float segMs = g_segLen / 44100.f * 1000.f;
        snprintf(l, sizeof(l), "Seg %.0f  %.0fms", g_segLen, segMs);
        oledStr(0, 33, l);

        snprintf(l, sizeof(l), "Rng T%.1f D%.1f M%d", g_tapeRange, g_drumRange, g_midiChannel);
        oledStr(0, 42, l);
        if (g_last_moved_pot < ANGLE8_POTS) {
            const char* kn = KnobNamesB[g_last_moved_pot];
            oledStr(127 - oledStrWidth(kn), 42, kn);
        }
        oledHLine(0, 50, 128);

        // Catch state: one marker per active Page-B pot (K0..K7), in the order
        // CV1s CV1o CV2s CV2o SEG TR DR MCH. '*' = caught (live), '.' = waiting.
        char cs[26];
        snprintf(cs, sizeof(cs), "%c%c %c%c %c %c%c %c", 
                 s_pbCaught[0] ? '*' : '.',
                 s_pbCaught[1] ? '*' : '.',
                 s_pbCaught[2] ? '*' : '.',
                 s_pbCaught[3] ? '*' : '.',
                 s_pbCaught[4] ? '*' : '.',
                 s_pbCaught[5] ? '*' : '.',
                 s_pbCaught[6] ? '*' : '.',
                 s_pbCaught[7] ? '*' : '.');
        oledStr(0, 54, cs);
    }

    // ── Bottom bypass bar ─────────────────────────────────────────────────────
    if (g_bypass) oledBox(0, 62, 128, 2);
    else          oledHLine(0, 63, 128);

    oledFlush();
}

// Display init + splash. Called once from setup() on the same core as loop().
void displayInit() {
    bool initok = oledInit();
    Serial.printf("[oled] init sequence: %s\n", initok ? "all ACKed" : "a command FAILED");

    oledClear();
    oledStr(37, 24, "ELTRO MK3");
    oledStr(40, 36, "STARTING...");
    oledFlush();
    Serial.println("[oled] first flush done");
}

// ═══════════════════════════════════════════════════════════════════════════════
// 7.  SETUP
// ═══════════════════════════════════════════════════════════════════════════════

void uiTask(void* arg);   // forward decl — defined after setup(), spawned in it

void setup() {
#if !ARDUINO_USB_CDC_ON_BOOT
    USB.begin();        // bring up native USB peripheral manually (see note above)
#endif
    Serial.begin(115200);
    delay(2000);         // give the CDC port time to enumerate before first print
    Serial.println("Eltro MK3");

    // TRS MIDI IN + OUT — AMY defines AMYBOARD_MIDI_IN = GPIO 21 (independent of
    // the front-panel I2C bus on 17/18). UART only, no I2C — safe before
    // amy_start(). V11: differential MIDI out (see ELTRO_MIDI_TX_PIN notes).
    //   TX (normal)  on ELTRO_MIDI_TX_PIN     — data / source leg
    //   TX (inverted) on ELTRO_MIDI_RETURN_PIN — active return leg (push-pull)
    // The inverted mirror is routed in hardware via the ESP32-S3 GPIO matrix, so
    // both pads emit the same UART1 TX signal, one inverted, perfectly aligned.
    Serial1.begin(31250, SERIAL_8N1, AMYBOARD_MIDI_IN, ELTRO_MIDI_TX_PIN);
    Serial1.setPins(AMYBOARD_MIDI_IN, ELTRO_MIDI_TX_PIN);  // force TX pad mux to GPIO14
    {
        // Route UART1 TX to the return pin, INVERTED, for active push-pull drive.
        uint32_t tx_sig = uart_periph_signal[1].pins[SOC_UART_TX_PIN_IDX].signal;
        gpio_set_direction((gpio_num_t)ELTRO_MIDI_RETURN_PIN, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal((gpio_num_t)ELTRO_MIDI_RETURN_PIN,
                                        tx_sig, true /*invert*/, false);
    }
    Serial.printf("MIDI pins: RX=%d TX=%d RET=%d(inv)\n",
                  AMYBOARD_MIDI_IN, ELTRO_MIDI_TX_PIN, ELTRO_MIDI_RETURN_PIN);

    // Load persisted Page-B settings (seg, ranges, CV) from NVS flash so they take
    // effect before first use. Defaults apply on a blank/first flash.
    loadPageBSettings();

    // V5: allocate the ring buffer in PSRAM (or SRAM fallback) before any audio or
    // head-position math. ringAlloc() also zeroes it.
    ringAlloc();

    // Non-I2C DSP state init — no bus traffic, safe before amy_start().
    // V8 two-axis init (ONCE):
    //   - grab playhead starts at HALF the buffer, as far as possible from the
    //     write head's start (0), giving the cold buffer room before read/write
    //     first cross. After init it moves freely at the tape rate.
    //   - each head's segment is grabbed at that start point;
    //   - the heads' scan phases are STAGGERED across the segment (h·seg/NUM_HEADS)
    //     so at any instant one head is mid-segment while another hands off —
    //     spreading the crossfades for a continuous output.
    {
        float seg = g_segLen;
        if (seg > (float)g_bufLen) seg = (float)g_bufLen;
        if (seg < (float)NUM_HEADS) seg = (float)NUM_HEADS;
        float startPos = (float)g_bufLen * 0.5f;
        s_grab = startPos;
        s_masterScan = 0.0f;
        float phaseStep = seg / (float)NUM_HEADS;
        for (int h = 0; h < NUM_HEADS; h++) {
            s_segOrigin[h]    = wrapf(startPos, (float)g_bufLen);
            // seed each head's previous derived phase to its staggered start so the
            // first-sample wrap detection doesn't false-trigger a grab.
            float p = phaseStep * (float)h;
            if (p >= seg) p -= seg;
            s_prevSegPhase[h] = p;
        }
    }
    hannLutInit();   // build the Hann window lookup table (DSP hot-path optimization)
    // (s_ring already zeroed by ringAlloc)

    // ─── CRITICAL ORDERING: amy_start() MUST run before ANY Wire/I2C activity ───
    // amy_start() configures the PCM9211/PCM5101 codecs over I2C and brings up the
    // I2S clock/framing from the *pristine power-on* bus. If the I2C bus has been
    // touched at all (Wire.begin, pot reads, LED or OLED writes) BEFORE amy_start(),
    // the I2S TX framing latches WRONG on cold power-up: analog out is loud,
    // distorted and level-independent even though the digital samples are clean.
    // That latch survives warm reset AND reflash (it lives in codec/I2S clock
    // state, not CPU state) and clears only on a power cycle or a TX-only I2S init.
    // This is the measured root cause of the long-standing cold-boot bug, isolated
    // by bisection: reordering alone (amy_start before I2C) fixes it. Do NOT move
    // any Wire/I2C call above amy_start().
    amy_config_t cfg = amy_default_config();
    cfg.features.default_synths  = 0;
    cfg.features.reverb           = 0;
    cfg.features.echo             = 0;
    cfg.features.chorus           = 0;
    cfg.features.startup_bleep    = 0;
    cfg.features.partials         = 0;
    cfg.max_oscs                  = 200;  // must exceed the audio-in osc index
                                          // (170); AMY's default is 250. Our
                                          // earlier cap of 1 made osc 170
                                          // unallocatable, so the analog input
                                          // was never routed and the hook barely
                                          // fired.
    // cfg.midi = AMY_MIDI_IS_NONE: AMY skips amy_arduino_usb_setup() (its competing
    // TinyUSB MIDI+CDC descriptors). Serial rides on the manual USBCDC object above;
    // MIDI IN is the TRS UART (Serial1), not USB MIDI. Verified independent of the
    // cold-boot bug — this USB/MIDI config cold-boots clean on its own.
    cfg.midi                      = AMY_MIDI_IS_NONE;
    cfg.features.audio_in         = 1;   // enable ADC input path
    amy_start(cfg);

    // ─── I2C bus init AFTER amy_start() (see ordering note above) ───
    // This is now the SOLE Wire bring-up. AMY has finished configuring the codecs
    // over the bus; we initialise it with our front-panel pins (SDA=17/SCL=18) and
    // the conservative 8Angle clock. ALL I2C — pots, LEDs, OLED — happens from here
    // on. (Replaces both the old pre-amy_start Wire.begin and the post-amy_start
    // Wire.end()/begin() "dance"; with nothing touching the bus before amy_start,
    // neither is needed.)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);   // 100kHz: conservative for 8Angle STM32 reliability (bus supports 400k)

    // Pots / LEDs — first read + seed of smoothing & display state.
    readAllPots();
    memcpy(pot_prev, pot_raw, sizeof(pot_raw));
    for (int i = 0; i < ANGLE8_POTS; i++) g_pot_display[i] = pot_raw[i];
    memset(led_flash_until, 0, sizeof(led_flash_until));
    // Page-B pots start "waiting" so the loaded/persisted values stay until a knob
    // is physically moved. Page A applies live as usual.
    rearmPageBCatch();
    applyPotsPageA();
    applyPotsPageB();   // catch-aware: leaves loaded values intact until pots move
    updateLeds(false, false);

    // OLED init — now after amy_start() on the freshly-begun bus (the same bus
    // state the runtime oledFlush() uses successfully). oledFlush() is error-checked
    // and skips a frame on any bus error rather than hanging.
    displayInit();

    // ── Audio in→out oscillators: official AMY external-buffer pattern ───────
    // synth 18, one voice, two oscillators — osc 0 plays the LEFT channel of the
    // external buffer (AUDIO_EXT0, panned hard left), osc 1 plays the RIGHT
    // (AUDIO_EXT1, panned hard right). We feed both channels the same processed
    // mono signal in loop(), so output is mono duplicated to L+R.
    //
    // amp_coefs[COEF_CONST] = 1.0 (unity): the Eltro's own input/output gain
    // knobs control level. (The official example uses 5.0 because AMYboard
    // audio-in is quiet, but we manage gain inside the effect instead.)
    {
        amy_event e = amy_default_event();
        e.reset_osc = RESET_AMY;
        amy_add_event(&e);

        e = amy_default_event();
        e.synth          = 18;
        e.num_voices     = 1;
        e.oscs_per_voice = 2;
        amy_add_event(&e);

        // osc 0 — left
        e = amy_default_event();
        e.synth                 = 18;
        e.osc                   = 0;
        e.wave                  = AUDIO_EXT0;
        e.pan_coefs[COEF_CONST] = 0.0f;   // hard left
        e.amp_coefs[COEF_CONST] = 1.0f;   // unity
        e.velocity              = 1.0f;
        amy_add_event(&e);

        // osc 1 — right
        e = amy_default_event();
        e.synth                 = 18;
        e.osc                   = 1;
        e.wave                  = AUDIO_EXT1;
        e.pan_coefs[COEF_CONST] = 1.0f;   // hard right
        e.amp_coefs[COEF_CONST] = 1.0f;   // unity
        e.velocity              = 1.0f;
        amy_add_event(&e);

        // Note-on both EXT oscillators to activate them.
        e = amy_default_event();
        e.synth = 18; e.osc = 0; e.wave = AUDIO_EXT0; e.midi_note = 60; e.velocity = 1.0f;
        amy_add_event(&e);
        e = amy_default_event();
        e.synth = 18; e.osc = 1; e.wave = AUDIO_EXT1; e.midi_note = 60; e.velocity = 1.0f;
        amy_add_event(&e);
    }

    // Spawn the UI/I2C task on core 0. loop() (core 1) is then audio-only.
    // 8KB stack: drawDisplay + Wire buffers + printf are comfortable within it.
    xTaskCreatePinnedToCore(uiTask, "ui", 8192, NULL, 1, NULL, 0);

    Serial.println("Ready  [BUILD: v10 — v9 + phase-locked heads (fixes buffer-change stutter): 2-axis Tempophon, 2xCV, MIDI RX, 20s PSRAM]");
}

// ═══════════════════════════════════════════════════════════════════════════════
// 8.  MAIN LOOP  (Arduino loopTask)
//     Audio: pull input block, run Eltro, push to external buffer, amy_update().
//     amy_update() blocks until the next audio block is ready, so it paces the
//     loop at the block rate. UI/I2C work is throttled to run every I2C_SCAN_MS.
// ═══════════════════════════════════════════════════════════════════════════════

// External-buffer scratch (interleaved stereo int16), reused every block.
// ═══════════════════════════════════════════════════════════════════════════════
// 8.  DUAL-CORE STRUCTURE
//     Core 1 (Arduino loop task): AUDIO ONLY — the external-buffer pump, back to
//        back, never blocked. Touches no I2C. This is what keeps the stream
//        continuous (the official example's loop does exactly this and nothing
//        else).
//     Core 0 (uiTask): ALL I2C + UI — pots, LEDs, OLED flush, MIDI/serial. It is
//        the SOLE owner of the Wire bus, so there is no cross-core I2C race and
//        no mutex is needed. Its slow ~100ms OLED flush no longer starves audio
//        because it runs on a different core.
//     Shared state (g_tapeSpeed, g_drumSpeed, g_bypass, …) is volatile and
//        word-sized — atomic on the ESP32 — so plain reads/writes are safe.
// ═══════════════════════════════════════════════════════════════════════════════

static output_sample_type s_extBuf[AMY_BLOCK_SIZE * AMY_NCHANS];

// ── UI / I2C task — runs forever on core 0 ───────────────────────────────────
void uiTask(void* arg) {
    uint32_t last_move_time = 0;
    bool     last_bypass    = false;
    uint32_t last_draw      = 0;
    uint32_t splash_done    = 0;
    uint32_t last_cv        = 0;

    for (;;) {
        uint32_t now = millis();

        // ── CV inputs (ADS1115 A0→CV1→tape, A1→CV2→drum), throttled ──────────
        // Non-blocking two-phase read; folds into the drum/pitch axis in the DSP
        // via effectiveDrum = drum + cvValue*cvScale + cvOffset. On the same bus
        // this task already owns, so no mutex needed.
        if (now - last_cv >= CV_READ_MS) {
            last_cv = now;
            serviceCvInput();
        }

        // ── Slider → bypass ──────────────────────────────────────────────────
        bool bypass = readToggle();
        if (bypass != last_bypass) {
            last_bypass = bypass;
            g_bypass    = bypass;
            updateLeds(g_bypass, g_pageB);
            Serial.printf("Bypass: %s\n", bypass ? "ON" : "OFF");
        }

        // ── Serial 'p' + MIDI CC 65 → page select ────────────────────────────
        while (Serial.available()) {
            char c = Serial.read();
            if (c == 'p') {
                bool wasB = g_pageB;
                g_pageB = !g_pageB;
                if (g_pageB && !wasB) rearmPageBCatch();   // entering B: pots start "waiting"
                if (!g_pageB && wasB) rearmPageACatch();   // entering A: pickup, don't snap
                if (!g_pageB && wasB && s_pbDirty) savePageBSettings();  // leaving B: persist now
                applyPots();
                updateLeds(g_bypass, g_pageB);
                Serial.printf("Page: %s\n", g_pageB ? "B" : "A");
            }
            // ── 'g<value>' → set speed-knob taper gamma (e.g. "g2.5") ─────────
            // Reads the digits/decimal point that follow 'g'. A bare 'g' just
            // prints the current value. Persisted to NVS immediately so it
            // survives a power cycle. Bounded to [SPEED_GAMMA_MIN, MAX].
            else if (c == 'g') {
                char numbuf[16];
                int  ni = 0;
                uint32_t t0 = millis();
                // Gather following number chars. Brief wait so a paste/typed
                // value that arrives a few ms later is still captured, without
                // blocking the task meaningfully.
                while (ni < (int)sizeof(numbuf) - 1) {
                    if (Serial.available()) {
                        char d = Serial.peek();
                        if ((d >= '0' && d <= '9') || d == '.' || d == '-') {
                            numbuf[ni++] = Serial.read();
                        } else break;
                    } else if (millis() - t0 > 8) {
                        break;   // no more chars coming
                    }
                }
                numbuf[ni] = '\0';
                if (ni > 0) {
                    float gv = atof(numbuf);
                    if (gv < SPEED_GAMMA_MIN) gv = SPEED_GAMMA_MIN;
                    if (gv > SPEED_GAMMA_MAX) gv = SPEED_GAMMA_MAX;
                    g_speedGamma = gv;
                    if (!g_pageB) applyPotsPageA();   // re-map tape/drum with new curve now
                    savePageBSettings();              // persist immediately
                    Serial.printf("Speed gamma: %.2f (saved)\n", g_speedGamma);
                } else {
                    Serial.printf("Speed gamma: %.2f\n", g_speedGamma);
                }
            }
        }
        // V9 MIDI RX: feed every available byte into the running-status-aware
        // state machine (handles CC65 page toggle + the Page-A CC map). Bounded
        // per pass so a flood can't starve the rest of uiTask.
        {
            int guard = 0;
            while (Serial1.available() && guard++ < 64) {
                midiFeedByte((uint8_t)Serial1.read());
            }
        }

        // ── Pots ─────────────────────────────────────────────────────────────
        readAllPots();
        bool any_changed = false;
        for (int ch = 0; ch < ANGLE8_POTS; ch++) {
            if (abs((int)pot_raw[ch] - (int)pot_prev[ch]) > POT_DEADBAND) {
                pot_prev[ch]        = pot_raw[ch];
                g_pot_display[ch]   = pot_raw[ch];
                led_flash_until[ch] = now + LED_FLASH_MS;
                g_last_moved_pot    = ch;
                last_move_time      = now;
                any_changed         = true;
            }
        }
        if (any_changed) {
            applyPots();
            if (!g_pageB) {
                updateLeds(g_bypass, false);
                midiTxPageA();   // V11: emit CCs for physically-moved Page-A knobs
            }
        }

        // ── Debounced persist of Page-B settings ─────────────────────────────
        // Save ~1s after the last change, so rapid knob sweeps coalesce into one
        // flash write (NVS has limited write-endurance). Page-switch handlers also
        // flush immediately on leaving Page B.
        if (s_pbDirty && s_pbDirtySince != 0 && (now - s_pbDirtySince) > 1000) {
            savePageBSettings();
            s_pbDirtySince = 0;
        }

        if (now - last_move_time > 1500) g_last_moved_pot = 255;

        // ── Per-LED flash restore ────────────────────────────────────────────
        for (int ch = 0; ch < ANGLE8_POTS; ch++) {
            if (now < led_flash_until[ch]) {
                setLed(ch, 255, 255, 255, 80);   // brief white flash on the touched knob
            } else {
                uint8_t r, g, b;
                if (g_bypass)     { r=200; g=0;   b=0;   }
                else if (g_pageB) { r=0;   g=200; b=0;   }
                else              ledColorForState(r, g, b);   // all 8 show effective pitch
                setLed(ch, r, g, b, 50);
            }
        }

        // ── Display ──────────────────────────────────────────────────────────
        if (splash_done == 0) splash_done = now + 1200;
        if (now >= splash_done && now - last_draw >= DISPLAY_MS) {
            last_draw = now;
            drawDisplay();   // includes the ~100ms OLED flush — fine on core 0
        }

        // Yield briefly so the scan rate is bounded and the idle task can run
        // (feeds the watchdog). I2C work above already takes several ms.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ── Audio pump — Arduino loop task, core 1, AUDIO ONLY ───────────────────────
void loop() {
    static uint32_t last_diag = 0;

    amy_get_input_buffer(s_extBuf);            // grab the ADC input block
    eltroProcessBlock(s_extBuf);               // downmix → Eltro → mono to L+R
    amy_set_external_input_buffer(s_extBuf);    // hand to AUDIO_EXT0/1 oscs
    amy_update();                               // render + I2S write; blocks ~1 block

    uint32_t now = millis();
    if (now - last_diag >= 1000) {
        last_diag = now;
        Serial.printf("[diag] blocks/s=%lu  inPeak=%ld  outPeak=%ld  bypass=%d\n",
                      (unsigned long)g_blockCount, (long)g_inPeak, (long)g_outPeak,
                      g_bypass ? 1 : 0);
        Serial.printf("       dsp: inGain=%.2f outGain=%.2f tape=%.2f drum=%.2f cv1=%.2f cv2=%.2f\n",
                      g_dbgInputGain, g_dbgOutputGain, g_dbgTape, g_dbgDrum, g_cv1Value, g_cv2Value);
        {
            char owned[ANGLE8_POTS + 1];
            for (int i = 0; i < ANGLE8_POTS; i++) owned[i] = s_midiOwned[i] ? ('0' + i) : '-';
            owned[ANGLE8_POTS] = '\0';
            Serial.printf("       midi: ch=%d owned=%s (CC102-109 -> knobs 0-7)\n", g_midiChannel, owned);
        }
        if (g_dumpReady) {
            Serial.print("       out16:");
            for (int k = 0; k < 16; k++) Serial.printf(" %6d", g_dumpBuf[k]);
            Serial.print("\n");
            g_dumpReady = 0;   // re-arm snapshot in eltroProcessBlock
        }
        g_blockCount = 0;
        g_inPeak     = 0;
        g_outPeak    = 0;
    }
}

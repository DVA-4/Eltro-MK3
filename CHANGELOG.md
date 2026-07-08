# Changelog

All notable changes to the Eltro MK3 firmware, newest first.

## V12 (current)
- Page-A knobs now use pickup ("catch") mode on page re-entry, matching
  Page B: returning to Page A no longer snaps a parameter to the pot's
  physical position. Page A is still live at boot.
- OLED MIDI-owned marker changed from `M` to `>`.

## V11
- MIDI TX added: the eight Page-A knobs emit their CCs (102–109) on physical
  movement, so a DAW/sequencer can record and play back knob motion.
- TX is driven differentially (see `docs/TECHNICAL_NOTES.md`) so it reaches
  DIN-4/5 MIDI interfaces cleanly.
- Guarded against RX/TX feedback: TX only fires on real pot movement and
  skips any knob currently MIDI-owned.

## V10
- Read heads phase-locked to a single master scan phase instead of four
  independent per-head accumulators. Fixes a bug where changing the buffer
  or segment length at runtime permanently collapsed the head stagger,
  causing a stutter. Steady-state sound is unchanged from V9.

## V9
- Two CV inputs added (CV1 → tape, CV2 → drum), read via an ADS1115.
- Full MIDI RX layer: CC control of all Page-A knobs with per-knob catch
  takeover.

## V8
- Core two-axis DSP established: progression (time) and pitch
  ((tape − drum)) as independent axes — the defining feature of the
  instrument. See `docs/TECHNICAL_NOTES.md` for the full model.

## Earlier
- V6/V7: single read-velocity precursors; V7 fixed a silence bug at
  tape == drum but still coupled pitch and progression to one number.

---

Hardware milestones:
- Cold-boot audio-in latch fixed — root cause was I2C access before
  `amy_start()`; see `docs/TECHNICAL_NOTES.md`.
- Independent knob-direction flag, on-screen output gain, decluttered
  gauge row.

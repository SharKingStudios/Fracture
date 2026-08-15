# MD_MIDIFile Notice

`MD_MIDIFile` 2.6.0 by Marco Colli / MajicDesigns is vendored here under LGPL-2.1 from:

https://github.com/MajicDesigns/MD_MIDIFile

Source revision: `ba4717e3c2ffb52de2fee10e7926138a1b9986d4`

Fracture-specific changes are intentionally confined to the vendored parser files:

- Replaced the SdFat file backend with Arduino `fs::FS` / `fs::File` so MIDI files can be read from ESP32 `FFat`.
- Added an absolute tick field to MIDI and metadata callbacks for chord grouping.
- Bounded metadata string termination.
- Kept full-path selection in the application instead of changing an SD-card working directory.

The complete upstream license is in `MD_MIDIFile.LICENSE`.
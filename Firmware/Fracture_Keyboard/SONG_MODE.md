# Song Learning Mode

## Arduino settings

The song drive requires a partition scheme containing a partition named `ffat` and TinyUSB/USB-OTG mode.

- Generic ESP32-S3 with 4 MB flash: use `Partition Scheme > No OTA (2MB APP/2MB FATFS)` and `USB Mode > USB-OTG (TinyUSB)`.
- Keep `USB CDC On Boot`, `USB Firmware MSC On Boot`, and `USB DFU On Boot` disabled. Fracture must configure USB itself before `USB.begin()` so normal MIDI and loader MSC do not race each other.
- The stock Arduino Nano ESP32 profile enables USB services before `setup()` and does not expose controls to disable all of them. It is therefore not supported by this composite MIDI/song-loader firmware; use an ESP32-S3 profile that exposes the settings above.
- Do not select a SPIFFS-only partition. When the real `ffat` storage cannot open, loader mode falls back to a small diagnostic USB drive named `SONG LOADER` with a `README.TXT`.

The first boot formats a blank FAT partition. Later mount failures are not auto-formatted, so an unsafe disconnect cannot silently erase stored songs.

## Controls

Normal mode:

- `S6`: enter song select.
- `S7`: refresh/reorder the connected boards.
- Encoder press: toggle the shared buzzer immediately.

Song modes:

- Encoder rotation: move through songs or the available hand choices.
- Encoder press: select/confirm.
- `S1`: escape/back. From the USB loader it safely leaves loader mode after eject/copying is complete.
- `S2` / `S3`: shared octave down/up by one octave. `S4`: reset to the base octave. `S5`: reveal the current title. Audio toggles from normal mode only.
- `S6`: intentionally does nothing while a song mode is active. It cannot delete songs.
- `S7` / `S8`: previous/next browser item or hand choice. If `LOAD SONG` is selected, `S8` enters the loader.

The first browser item is always `LOAD SONG`. Every other entry is collected from the FAT storage on every connected board, so a song stored on one board can be selected from any board.

## Loading files

1. Connect a USB **data** cable directly from the computer to the board on which you want to store the file. The inter-board link does not expose a computer drive.
2. Press `S6` on that board, leave `LOAD SONG` selected, and press the encoder or `S8`. The selected board turns green and the other boards remain orange, so the USB target is unambiguous.
3. The selected board restarts into loader mode and mounts its **actual song library** as the `Fracture Songs` drive. It remains a MIDI device while the drive is present; normal boots expose MIDI only. The restart is required so USB can enumerate the storage interface safely.
4. Use the computer's file explorer to add, rename, or delete Standard MIDI files (`.mid` or `.midi`, format 0 or 1). The current songs are visible immediately; there is no separate staging area.
5. Eject the drive in the operating system. If eject is unavailable, wait for copying to finish and press `S1`.

The loader gives the computer exclusive access to that board's song library. Always eject before pressing `S1` or disconnecting USB; after exit, the firmware restarts, rescans the files, sends a green update wave from the leftmost key to the rightmost key, and rebuilds the distributed song list.

Outside song-menu navigation, encoder rotation changes the shared octave in one-octave steps. In the song browser and hand selector, it moves the menu selection instead.

The loader presents the board's existing FFat song volume directly, using its native wear-leveling geometry. Do not open or change files on both the board and computer at once; loader mode makes the board's local filesystem inactive until it exits.

## Learning behavior

Notes sharing the same MIDI tick are treated as a chord. The guide remains on that chord until every required note has been pressed; wrong notes do not advance it.

- Left hand: green.
- Right hand: red.
- Unknown hand: red.

Hand data is accepted when all note streams have explicit track names such as `Left`, `LH`, `Bass`, `Right`, `RH`, or `Treble`. For an unnamed two-stream piano file, the lower-average stream is inferred as left and the higher-average stream as right. The assignment stays attached to the whole MIDI stream, so either hand may cross the full keyboard. If the assignment is not reliable, the hand selector is skipped and every guide key is red.

Current limits are 16 files per board, 64 files across the chain, and 12,000 note-on events in the loaded song. Browser titles are shortened to 24 characters for the RS-485 packet.

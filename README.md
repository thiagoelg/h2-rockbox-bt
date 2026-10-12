# Rockbox with Bluetooth for the HiFi Walker H2

This is a fork of [Rockbox](https://www.rockbox.org/) that adds Bluetooth
audio output to the native Rockbox port for the **HiFi Walker H2**: the AIGO
Eros Q family's "erosqnative" target, hardware revision 4.

> **Unofficial and experimental.** This fork is not affiliated with or
> endorsed by the Rockbox project. Please don't report problems with it to
> the Rockbox developers. Use it at your own risk.

## Compatibility

Tested on this setup:

| | |
| --- | --- |
| Player | HiFi Walker H2 |
| Original firmware | H2 firmware V2.3 (`H2-Firmware-V2.3.upt`) |
| Hardware revision | 4 |
| Rockbox bootloader | erosqnative hardware revision 4 bootloader (`erosqnative-hw4-erosq_2024.upt`) |
| Bluetooth module | AzureWave AW-NM372 (Cypress CYW43438 / BCM4343A1) |
| Rockbox | master as of 2026-10-11, newer than the 4.0 release |

- **Check the hardware revision** in Rockbox under **System > Debug (Keep
  Out) > Device data**: it must say "Hardware Revision: 4".
- **Other players that use the erosqnative build** run this firmware like
  regular Rockbox, but Bluetooth is only enabled on hardware revision 4, and
  has only been tested on the H2.
- **This is a complete Rockbox build.** It replaces the Rockbox already on
  the player, including its codecs and plugins. Your settings are kept. The
  bootloader isn't changed.

## What works

- Pairing and connecting to Bluetooth headphones and speakers, using A2DP
  with the SBC codec.
- A **High Quality Audio** setting that uses SBC dual channel at the highest
  bitpool the headphones allow ("SBC XQ", for example about 441 kbps instead
  of about 240 kbps).
- **Volume sync** using AVRCP absolute volume: the H2's volume buttons
  control the headphones' volume, and the headphones' volume controls move
  the H2's volume bar.
- Play, pause, next, previous, fast forward and rewind from the headphones'
  buttons.
- Reconnecting automatically to the last device used, and paired headphones
  connecting on their own.
- Following the **Pause on Headphone Unplug** setting when the headphones
  disconnect.
- Powering the H2's headphone amplifier down while playing over Bluetooth.

It has been tested with Samsung Galaxy Buds Pro and Bose QC45 headphones.

## Limitations

- Only SBC. AAC, aptX and LDAC aren't supported.
- The stream always runs at 44.1 kHz. 48 kHz material is resampled.
- One device at a time. Audio output only: no receiving audio, no calls or
  microphone, no Bluetooth LE.
- Only hardware revision 4 has a Bluetooth driver. On earlier Eros Q
  revisions, turning Bluetooth on reports that it failed to start.
- Bluetooth uses more battery while it's on, even when nothing is
  connected.

## Using it

Bluetooth lives in **Settings > Bluetooth**:

- **Bluetooth**: On or Off.
- **Devices**: paired devices, followed by nearby devices that are in
  pairing mode. Select a device to connect or disconnect. Use the context
  menu to forget a paired device. The first line shows the status; select
  it to search again, or to turn Bluetooth on.
- **High Quality Audio**: SBC dual channel when the headphones support it.
  Turn it off if playback stutters.
- **Sync Volume with Headphones**: AVRCP absolute volume, when the
  headphones support it.

The manual's new Bluetooth chapter (`manual/configure_rockbox/bluetooth.tex`)
covers the same ground.

## Installing

The player needs the Rockbox bootloader first. The installation chapter of
the [Rockbox manual](https://www.rockbox.org/manual.shtml) for the AIGO Eros
Q native port covers that.

Then download `rockbox-h2-bt-<version>.zip` from the
[Releases](../../releases) page. Extract it to the root of the SD card,
replacing the existing `.rockbox` directory, so that the firmware matches its
plugins and codecs.

Each release also has a `-debug` zip, which logs Bluetooth activity (see
[Debugging](#debugging)). Only use it to investigate problems.

## Building

GitHub Actions builds every push to `h2-bt` (download the zips from the
workflow run's artifacts) and publishes a release for every tag starting
with `v`. To build locally instead, build it like any Rockbox target:

```sh
# Once: build the MIPS toolchain (pick the MIPS target when asked)
tools/rockboxdev.sh

mkdir build && cd build
../tools/configure --target=erosqnative --type=n
make -j"$(nproc)"
make zip
```

Then install `rockbox.zip` as described above.

### Debugging

The `-debug` builds log the Bluetooth stack and the integration code to
`/bt_log.txt`, and capture HCI traffic to `/hci_log.pcap`, which opens in
Wireshark. The previous logs are kept as `*.old.*`. To get the same from a
local build, uncomment `#define BT_DEBUG` in
`firmware/export/config/erosqnative.h`.

## How it's organised

The `h2-bt` branch is a series of commits on top of Skye Green's Bluetooth
work for Rockbox, rebased onto Rockbox master as of 2026-10-11 (`f282ae203`).
Each commit explains what it changes and why. The series:

| Area | Commits |
| --- | --- |
| Skye's series | Bluedroid import and port, the `bt_hci` interface, the erosqnative HCI driver, the integration code |
| Fixes to the X1000 UART driver | `uart_rx_timeout()`, interrupt races |
| Fixes to Bluedroid | Work queue overflow and thread joins, packets after shutdown, Just Works pairing, SBC encoder guards, the SBC XQ bitrate, restarting the stack |
| erosqnative | HCI driver rework (power sequence, 32 kHz sleep clock, bring-up timeouts, 3 Mbaud, framing fixes), headphone amp shutdown |
| Simulator | The simulator HCI driver builds without SDL threads |
| Apps | Manager thread, device list, settings, volume sync, unplug handling |
| Manual | A Bluetooth chapter |

The code follows the Rockbox coding guidelines (`docs/CONTRIBUTING`), so that
parts of it can be proposed upstream later. Only this README and the
GitHub Actions workflow (`.github/workflows/build.yml`) are specific to the
fork, in the two commits at the top.

## Credits

- **The Rockbox developers.** Everything here is built on more than two
  decades of their work. See `docs/CREDITS` for the full list of
  contributors.
- **Skye Green** did the foundational work: importing and porting Bluedroid
  to Rockbox, designing the `bt_hci` interface, writing the HCI driver for
  the erosqnative hardware revision 4, and the first integration of A2DP and
  AVRCP with Rockbox's audio output. Without that work this fork wouldn't
  exist. Their series is under review on the Rockbox Gerrit:
  - [7393: bt: import Bluedroid from esp-idf 6.0.1](https://gerrit.rockbox.org/r/c/rockbox/+/7393)
  - [7384: bt: port Bluedroid for Rockbox](https://gerrit.rockbox.org/r/c/rockbox/+/7384)
  - [7497: bt: add bt_hci interface](https://gerrit.rockbox.org/r/c/rockbox/+/7497)
  - [7499: erosqnative: implement bt_hci for hw4 only](https://gerrit.rockbox.org/r/c/rockbox/+/7499)
  - [7386: bt: enable bluetooth for erosqnative](https://gerrit.rockbox.org/r/c/rockbox/+/7386)
  - [7387: bt: WIP integration code](https://gerrit.rockbox.org/r/c/rockbox/+/7387)
- **Bluedroid**, the Bluetooth stack, comes from Espressif's
  [ESP-IDF](https://github.com/espressif/esp-idf) v6.0.1, which in turn
  derives from Android's Bluedroid by Broadcom and Google. See
  `lib/bluedroid/README`.

## AI usage disclosure

I developed the changes on top of Skye Green's series with **Claude**
(Anthropic's AI model; Claude Opus 5.5 in Claude Code). Claude did most of
the work:

- reverse engineering the original firmware for the hardware details
- writing and reviewing the code
- analysing logs and HCI captures
- structuring the commits

I defined the goals, tested every build on the H2 with real headphones,
reported the problems I found, and reviewed the results.

Every commit written this way carries a `Co-Authored-By: Claude` trailer.
Skye's commits, Bluedroid and the rest of Rockbox are not covered by this
disclosure.

Rockbox's contribution guidelines (`docs/CONTRIBUTING`) say the project is
highly unlikely to accept AI-generated code. Code that is submitted must
document its provenance, be structured in logical commits, and come from a
contributor prepared to justify every change. I'm publishing this fork in
that spirit, and I won't propose anything from it upstream until I
understand it well enough to meet those requirements.

## License

Rockbox is free software, licensed under the GNU General Public License,
version 2 or (at your option) any later version. See `docs/COPYING`. The
Bluedroid files under `lib/bluedroid` keep the licences stated in their
headers, mostly the Apache License 2.0.

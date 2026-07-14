# kv4p RX streamer — ESP32 firmware (Rust)

RX-only WiFi audio streaming firmware for the kv4p-ht board
(ESP32-WROOM-32 + SA818). This is a Rust (`std`, ESP-IDF) rewrite of the
Arduino C++ firmware in `../../kv4p_rx_streamer/`; that C tree remains the
behavioral reference.

The board is permanently in RX (PTT never asserted). It serves live PCM16 mono
16 kHz audio over HTTP as an endless WAV, hosts a web UI for configuration, and
optionally decodes VDV-FFSK / NEMO telegrams and forwards them to a backend.

## Repository layout

```
rx-streamer-rs/
  firmware/      <- this crate (ESP32 binary)
  nemo-decode/   <- NEMO telegram decoder crate (owned separately)
  ../../../ffsk-decode   <- FFSK/VDV decoder, a SIBLING checkout of the
                            bielefeld-live backend's `ffsk-decode` crate
```

The decoder crates are wired into the firmware in a later phase. The FFSK
decoder is expected as a sibling checkout so both the backend and this firmware
build against the same `ffsk-decode` source; the path dependency lands with the
decoder module.

## Prerequisites

Install the Espressif Rust toolchain and the build shims:

```sh
cargo install espup ldproxy espflash
espup install            # installs the `esp` Xtensa toolchain + LLVM
. ~/export-esp.sh        # sets IDF/toolchain env for the current shell
```

`rust-toolchain.toml` pins `channel = "esp"`, so `cargo` in this directory uses
the Xtensa toolchain automatically. The first build downloads and builds
ESP-IDF v5.3.x via `embuild` (several minutes).

## Build / flash / monitor

```sh
cargo build                     # debug
cargo build --release           # optimized (opt-level "s", fat LTO)
cargo run --release             # flash + serial monitor (see runner below)
```

`cargo run` uses the configured runner:
`espflash flash --monitor --partition-table partitions/min_spiffs.csv`.

## Compatibility guarantees

* **NVS is byte-compatible with the C firmware.** Settings live in the
  `rxstream` namespace and the channel table in the `channels` blob, using the
  exact keys, primitive types and packed struct layouts the Arduino
  `Preferences` firmware wrote (see `src/config.rs`). Booting Rust on a board
  previously running the C firmware preserves WiFi, admin password, tuning and
  channels. The size-keyed v1→v2 channel-table migration is preserved.
* **The board description is read-only.** The `hwconfig` namespace (factory pin
  map / ADC settings / RF module type) is only read, never written, exactly as
  in C. Strap-pin revision detection (GPIO39/36) is unchanged.
* **The partition table is byte-identical** to the C build's `min_spiffs.csv`
  (two ~1.9 MB OTA slots), so an OTA image built here fits the same layout:
  `nvs 0x9000/0x5000, otadata 0xe000/0x2000, app0 0x10000/0x1E0000,
  app1 0x1F0000/0x1E0000, spiffs 0x3D0000/0x30000`.

## Module map (C reference → Rust module)

| Rust module     | C reference        | Responsibility                          |
|-----------------|--------------------|-----------------------------------------|
| `src/board.rs`  | `hardware.h`       | pin map, ADC settings, board revision   |
| `src/config.rs` | `config.h`         | NVS persistence, channel table, sharing |
| `src/radio.rs`  | `radio.h`          | SA818 AT driver (UART1)                  |
| `src/audio.rs`  | `audio.h`          | ADC DMA capture + DSP (DC/gain/FIR)      |
| `src/wifi.rs`   | `wifiMgr.h`        | STA/AP, mDNS, SNTP                       |
| `src/rt.rs`     | (infra)            | pinned thread spawn, WDT, dynamic GPIO   |
| `src/main.rs`   | `kv4p_rx_streamer.ino` | setup + core-0 supervisor loop      |

Later phases add `streamer.h`, `webui.h`, `decoder*.h`, `uplink.h`,
`otaUpdate.h` at the spawn points marked in `main.rs`.

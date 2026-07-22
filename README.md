# SkyReach
SkyReach is an open-source handheld drone detection platform. It passively receives Wi-Fi and Bluetooth Open Drone ID broadcasts, decodes ASTM F3411 Remote ID messages, logs detections to SD, and displays nearby drones in a lightweight on-device interface. Future expansions add GNSS, LoRa, ExpressLRS, SiK support.
## Phase 1 — Wi-Fi/BLE Open Drone ID Detector

A passive Remote ID (ASTM F3411 / ODID) receiver that listens for the mandatory Wi-Fi and BLE broadcasts drones emit, and logs everything it can decode for later analysis.

### Hardware

- **[Kode Dot](https://github.com/kodediy)** — ESP32-P4 (application processor, 32 MB flash / 32 MB PSRAM, MIPI DSI display) + ESP32-C5 (wireless co-processor: Wi-Fi 2.4/5 GHz, BLE)
- MicroSD card (local storage)
- RTC (MAX31329) for reliable timestamps

### Libraries

| Need | Library |
|---|---|
| BLE scanning | [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) |
| Wi-Fi promiscuous mode | ESP-IDF `esp_wifi.h` (built into the Arduino core) |
| ODID decoding | [opendroneid-core-c](https://github.com/opendroneid/opendroneid-core-c) |
| JSON/NDJSON | [ArduinoJson v7](https://arduinojson.org) |
| SD storage | `SD_MMC.h` / `SD.h` |

### Architecture

```
ESP32-C5                                    ESP32-P4
─────────                                   ─────────
BLE scan (NimBLE) ─┐
                    ├─► ODID filtering ─► IPC ─► decode (opendroneid-core-c)
Wi-Fi promiscuous ──┘     (Company ID /          ─► state merge (drone_entry_t)
                           vendor OUI)            ─► SD logging (NDJSON, buffered)
                                                   ─► LVGL UI (list / detail / status)
```

Detections are merged by source MAC into a single per-drone state, since the same drone often splits its message types across separate packets (particularly in BLE Legacy Advertising mode).

### Decoded messages — Wi-Fi and BLE

**Raw data identification:**

| Radio | Location | Identifier |
|---|---|---|
| BLE Advertising | AD type `0xFF` (Manufacturer Specific) | Company ID `0xFA0A` |
| Wi-Fi Beacon | Information Element `0xDD` (Vendor Specific) | OUI `F1:F0:04` |

**Decoded ODID message types (all of them, no pre-filtering):**

| Message | Contents |
|---|---|
| **Basic ID** | UA type, ID type (serial number / CAA registration / UTM UUID / session ID), the identifier itself — may appear multiple times per drone |
| **Location/Vector** | lat/lon, barometric and geodetic altitude, height AGL, direction, horizontal/vertical speed, accuracy classes, timestamp, flight status (ground/airborne/emergency) |
| **Auth** | authentication data by page, if the transmitter supports it |
| **Self ID** | free-text operator description |
| **System** | operator location, EU category/class, area classification |
| **Operator ID** | operator registration ID |

All missing/optional fields (Auth, Self ID, System) are explicitly flagged as "not broadcast" rather than assumed to be zero — many manufacturers omit these.

### Library choices — rationale

| Library | Chosen over | Why |
|---|---|---|
| NimBLE-Arduino | Bluedroid (default Arduino BLE stack) | ~50% less flash, ~100 KB less RAM, native BLE5 Extended Advertising support |
| opendroneid-core-c | Custom parser | Standard-maintainer's own reference implementation, MIT-licensed, no reason to reinvent bit-level ASTM F3411 parsing |
| ArduinoJson v7 | Hand-built string concatenation | Automatic escaping (operator free-text fields), dynamic `JsonDocument`, no fixed-buffer overflow risk |
| SD_MMC (SDIO) | SD (SPI) | Significantly faster writes — relevant because Phase 1 logs everything unfiltered |

Since Phase 1's design decision is to log *all* raw data without pre-filtering, each choice favors memory safety and write throughput over raw performance.
# SkyReach
SkyReach is an open-source handheld drone detection platform. It passively receives Wi-Fi and Bluetooth Open Drone ID broadcasts, decodes ASTM F3411 Remote ID messages, logs detections to SD, and displays nearby drones in a lightweight on-device interface. Later phases add GNSS (receiver position) and, on separate radio hardware, RF presence detection of LoRa, ExpressLRS and SiK links (see [Roadmap](#roadmap)).
## Phase 1 — Wi-Fi/BLE Open Drone ID Detector

A passive Remote ID (ASTM F3411 / ASD-STAN prEN 4709-002) receiver that listens for all four broadcast methods defined by the standard and logs both the raw frames and everything it can decode, for later analysis.

### Hardware

- **[Kode Dot](https://github.com/kodediy)** — ESP32-P4 (application processor, 32 MB flash / 32 MB PSRAM, MIPI DSI display) + ESP32-C5 (wireless co-processor: Wi-Fi 2.4/5 GHz, BLE 5)
- MicroSD card (local storage)
- RTC (MAX31329) for reliable timestamps

### Broadcast methods

ASTM F3411 defines four broadcast methods. Manufacturers pick one or more, so the receiver must cover all of them:

| Method | Radio / PHY | Channel | Payload |
|---|---|---|---|
| BT4 Legacy Advertising | BLE 1M PHY, legacy advertising | BLE advertising channels 37/38/39 | one 25-byte message per advertisement |
| BT5 Long Range | BLE Coded PHY, extended advertising | BLE advertising channels | Message Pack |
| Wi-Fi NAN | Service Discovery action frame | 2.4 GHz ch 6, 5 GHz ch 149 | Message Pack |
| Wi-Fi Beacon | Beacon frame, vendor-specific IE | any channel the transmitter uses, 2.4 or 5 GHz | Message Pack |

BLE scanning therefore runs both a legacy 1M scan and an extended Coded PHY scan. NimBLE-Arduino support for Coded PHY scanning on the C5 is still to be verified.

### Raw frame identification

| Method | Location | Identifier | Followed by |
|---|---|---|---|
| BLE (BT4 / BT5) | AD type `0x16` (Service Data — 16-bit UUID) | UUID `0xFFFA` + AD Application Code `0x0D` | 1-byte message counter, then the message (BT4) or Message Pack (BT5) |
| Wi-Fi Beacon | Information Element `0xDD` (Vendor Specific) | OUI `FA:0B:BC` + `oui_type` `0x0D` | 1-byte message counter, then Message Pack |
| Wi-Fi NAN | Action frame to `51:6F:9A:01:00:00`, category `0x04`, action `0x09` | Wi-Fi Alliance OUI `50:6F:9A` type `0x13`, service ID `88:69:19:9D:92:09` | Service Descriptor attribute with message counter and Message Pack |

The message counter must be skipped before decoding; it is also used to drop duplicate frames.

> The identifiers match opendroneid's reference transmitters: [`transmitter-linux/bluetooth.c`](https://github.com/opendroneid/transmitter-linux) (BLE) and [`opendroneid-core-c/libopendroneid/wifi.c`](https://github.com/opendroneid/opendroneid-core-c) (Wi-Fi). This only shows that the constants agree with the same code base. Interoperability must be validated against real captures (DJI, add-on broadcast modules), see [Validation](#validation).

### What opendroneid-core-c does and does not do

| Step | Provided by the library | SkyReach implements |
|---|---|---|
| Wi-Fi NAN frame → messages | `odid_wifi_receive_message_pack_nan_action_frame()` | — |
| Wi-Fi Beacon frame → messages | no receive function (build only) | IE walk, OUI/type check, skip counter, then `odid_message_process_pack()` |
| BLE advertisement → messages | nothing | AD parsing, UUID/app-code check, skip counter, then `decodeOpenDroneID()` (BT4) or `odid_message_process_pack()` (BT5) |
| Single message / Message Pack decoding | `decodeOpenDroneID()`, `decodeMessagePack()` | — |
| Per-drone state across packets | no, `odid_message_process_pack()` re-initialises its `ODID_UAS_Data` on every call | merge into `drone_entry_t` |
| Protocol version / plausibility checks | no, version is not checked and any value 0–15 is accepted for enums | log `ProtoVersion` per message, flag implausible values |

Build options: compile with `-DODID_BASIC_ID_MAX_MESSAGES=5` (the library maximum). With the default of 2, a third Basic ID type is silently dropped and a Message Pack containing more Basic IDs than the limit is rejected entirely.

The library also contains `libopendroneidcn` (China GB 46750-2025, a separate wire format) and a French FRDID beacon builder (transmit only). Neither is ASTM F3411 and neither is in scope for Phase 1.

### Architecture

Open decision, to be settled first: the Kode Dot C5 runs esp-hosted firmware by default. Through esp-hosted the P4 gets normal Wi-Fi and BLE, but Wi-Fi promiscuous callbacks are not expected to reach the P4 (to be verified), and the BLE host stack (NimBLE) runs on the P4 with only the controller on the C5. Options:

| Option | C5 firmware | Effect on P4 | IPC |
|---|---|---|---|
| A | esp-hosted fork with an added RPC event for sniffed frames | keeps normal Wi-Fi/BLE | extends the existing esp-hosted transport |
| B | custom SkyReach firmware (sniffer + BLE scanner + filtering) | loses normal Wi-Fi/BLE | custom: link (SPI/SDIO/UART), frame format, flow control |

The intended data flow, independent of where the split ends up:

```
ESP32-C5                                       ESP32-P4
─────────                                      ─────────
BLE scan (1M legacy + Coded ext.) ─┐
                                    ├─► ODID filtering ─► IPC ─► raw frame log (hex)
Wi-Fi promiscuous (Beacon + NAN) ──┘    (UUID 0xFFFA /         ─► frame parsing + decode (opendroneid-core-c)
  channel hopping                        OUI FA:0B:BC /         ─► state merge (drone_entry_t)
                                         NAN service ID)        ─► SD logging (NDJSON, buffered)
                                                                ─► LVGL UI (list / detail / status)
```

### Radio scheduling

The C5 has a single radio shared by Wi-Fi (2.4 and 5 GHz) and BLE (2.4 GHz, time-shared through coexistence). Detection probability is set by the channel plan, which is still to be designed:

- NAN on ch 6 and ch 149
- Beacon on whichever channel the transmitter chose, in either band
- BLE scan windows (legacy and Coded) interleaved with Wi-Fi dwell time

Dwell times, the hopping order and the Wi-Fi/BLE time split must be defined and measured.

### State merge

The same drone sends from different MAC addresses over Wi-Fi and BLE, BLE addresses can rotate, and in BT4 Legacy mode Location packets carry no UAS ID. Merging is therefore two-level:

1. Per transport: source MAC + broadcast method → links Location, System, etc. to the Basic ID seen from the same address.
2. Across transports: UAS ID (Basic ID) is the primary key; a drone seen on several methods becomes one entry.

A drone may send up to five Basic ID messages with different ID types; all are kept.

### Decoded messages

All message types are decoded, no pre-filtering:

| Message | Contents |
|---|---|
| **Basic ID** (0) | UA type, ID type (serial number / CAA registration / UTM UUID / specific session ID), the identifier itself — may appear multiple times per drone with different ID types |
| **Location/Vector** (1) | status (undeclared / ground / airborne / emergency / Remote ID system failure), lat/lon, barometric and geodetic altitude, height with reference (over takeoff / over ground), direction, horizontal/vertical speed, accuracy classes, timestamp (tenths of seconds after the full hour) with accuracy |
| **Auth** (2) | authentication type and data, up to 16 pages; page 0 carries length and a timestamp |
| **Self ID** (3) | description type (text / emergency / extended status) and 23-character description |
| **System** (4) | operator location type (takeoff / live GNSS / fixed), operator location and altitude, group operation area (count, radius, ceiling, floor), classification type, EU category/class, timestamp (seconds since 2019-01-01 UTC) |
| **Operator ID** (5) | operator registration ID |
| **Message Pack** (0xF) | container of up to 9 of the above; used by BT5, NAN and Beacon |

Basic ID, Location/Vector and System are required by the FAA rule; the EU additionally requires Operator ID. Self ID and Auth are optional. A drone that never sends a mandatory message is flagged as a finding of its own.

Each field is recorded as one of:

- not broadcast: the message containing it has not been received
- broadcast, unknown: the transmitter sent the standard's "invalid / no value" sentinel (lat/lon 0, altitude −1000 m, direction 361°, horizontal speed 255 m/s, vertical speed 63 m/s, timestamp 0xFFFF)
- valid value

Sentinels are never shown as real values (e.g. a drone at 0°, 0°).

### Time

The Location timestamp only covers the position within the current hour; the hour comes from the receiver clock. Before GNSS is available the MAX31329 RTC is the time source. RTC initialisation and drift handling are still open; the System message timestamp (absolute, seconds since 2019-01-01) can be used as a cross-check.

### Logging

- Every received ODID frame is logged as raw hex alongside the decoded result, so parser bugs and new message types can be reprocessed later.
- Each record includes receive time, broadcast method, channel/PHY, RSSI, source MAC, message counter and `ProtoVersion`.
- Still to be defined: flush policy (data loss on power cut) and log rotation (FAT32 4 GB file limit).

### Security and privacy

- Auth messages are decoded but not verified, so all received data can be spoofed. Detections are reports of what was broadcast, not proof of identity or position.
- Operator location and operator registration ID are personal data under the GDPR. Logs that are shared must be handled accordingly.

### Validation

Round-trip tests against opendroneid's encoder only prove symmetry with the same code base. Interoperability is validated against real captures: DJI drones, add-on broadcast modules, and existing receivers such as [opendroneid receiver-android](https://github.com/opendroneid/receiver-android) and the [Wireshark dissector](https://github.com/opendroneid/wireshark-dissector). [sxjack/uav_electronic_ids](https://github.com/sxjack/uav_electronic_ids) contains ESP32 receive code for comparison.

### Library choices — rationale

| Need | Library | Chosen over | Why |
|---|---|---|---|
| BLE scanning | [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) | Bluedroid (default Arduino BLE stack) | native BLE5 extended advertising and PHY selection; ~50% less flash and ~100 KB less RAM, which matters on whichever chip ends up running the host stack (much less on the P4 with 32 MB PSRAM) |
| Wi-Fi promiscuous mode | ESP-IDF `esp_wifi.h` | — | runs on the C5 (option B) or through an esp-hosted extension (option A) |
| ODID decoding | [opendroneid-core-c](https://github.com/opendroneid/opendroneid-core-c) | custom parser | the standard maintainers' own reference implementation, Apache-2.0 licensed (its license text ships with SkyReach binaries); builds for ESP32 (`ARDUINO_ARCH_ESP32` / `IDF_VER` support in `wifi.c`) |
| JSON/NDJSON | [ArduinoJson v7](https://arduinojson.org) | hand-built string concatenation | automatic escaping (operator free-text fields), dynamic `JsonDocument`, no fixed-buffer overflow risk |
| SD storage | `SD_MMC.h` (SDIO) | `SD.h` (SPI) | significantly faster writes — relevant because Phase 1 logs every raw frame |

## Roadmap

- GNSS: receiver position and absolute time (replaces the RTC as time source).
- RF presence detection: LoRa, ExpressLRS and SiK control/telemetry links. These need separate radio hardware and are not Remote ID. They reveal that a transmitter is present, not who or where the drone is.

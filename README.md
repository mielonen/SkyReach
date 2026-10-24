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
| Wi-Fi Beacon | Beacon frame, vendor-specific IE | operating channel or social channel (ch 6 / ch 149) | Message Pack |

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

The C5 has one radio for Wi-Fi (2.4 and 5 GHz) and BLE. Espressif lists Wi-Fi sniffer RX together with BLE scan as supported but unstable on the C5 ([RF coexistence](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c5/api-guides/coexist.html)), and the coexistence guide does not cover 5 GHz. SkyReach therefore does not run both at once. A scheduler switches between Wi-Fi and BLE slots: the sniffer is off during BLE slots and BLE scanning is stopped during Wi-Fi slots.

#### What the plan is based on

- Dynamic messages are sent at 1 Hz or faster, static messages at least every 3 s (EN 4709-002).
- NAN uses ch 6 and ch 149.
- A Beacon is sent either on the transmitter's operating channel or on a social channel (ch 6 or ch 149). The Message Pack is typically repeated about once per second. Not confirmed from the standard text.
- BLE legacy advertising uses ch 37/38/39. BT5 Long Range uses Coded PHY extended advertising, with the payload on secondary channels. The controller follows the aux pointers.
- F3411-19 requires BT5 Long Range to be sent together with BT4 Legacy. If this still holds in F3411-22a, a legacy scan finds every BLE transmitter and the Coded scan only adds range.

Dwell time has to match the transmit interval. A 200 ms dwell catches a 1 Hz beacon in only about 20 % of visits, so short-dwell hopping over all channels is not enough.

#### Schedule

The base cycle is 3 s, matching the static message period. The slot lengths below are starting values to be tuned from measurements.

| Slot | Length | Covers |
|---|---|---|
| Wi-Fi ch 6 | 900 ms | NAN and social-channel Beacon, 2.4 GHz |
| BLE | 900 ms | legacy scan; Coded scan in alternate cycles |
| Wi-Fi ch 149 | 700 ms | NAN and social-channel Beacon, 5 GHz |
| Search | 500 ms | one other channel per cycle, rotating over 2.4 GHz ch 1–13 and 5 GHz UNII-3 |

With these values a 1 Hz transmitter is caught in roughly 70–90 % of cycles, giving a location update about every 3 s.

The search slot is adaptive. A Beacon found outside the social channels adds its channel to a tracked list, together with the measured interval, and the search slot visits tracked channels first. A tracked channel with no ODID frames for 30 s is dropped.

Frames from the same drone can arrive on several channels and methods. Duplicates are removed by message counter before state merge, and the scheduler does not count them as new detections.

#### Metrics

Each schedule is evaluated with:

- time to first detection
- location update interval per drone
- missed frame rate per broadcast method, against a reference transmitter with a known interval

#### Open items to measure

1. Sniffer + BLE scan on the C5: frame loss with the built-in coexistence compared with the software slot scheduler.
2. Channel switch time within 2.4 GHz and between bands. With esp-hosted (option A) the switch is an RPC from the P4, which adds latency.
3. Whether ch 149 can be selected under the configured country code (`esp_wifi_set_country`), even for receive only.
4. Beacon interval and channel of real transmitters (DJI, add-on modules). Reference transmitters: opendroneid [transmitter-linux](https://github.com/opendroneid/transmitter-linux) or the ESP32 transmitter in [uav_electronic_ids](https://github.com/sxjack/uav_electronic_ids).
5. Whether BT5 transmitters always send legacy advertisements as well. If not, the Coded scan needs its own slot every cycle.

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

## References

Standards (not freely available):

- ASTM F3411-22a, Standard Specification for Remote ID and Tracking
- ASD-STAN prEN 4709-002, Direct Remote Identification

Code:

- [opendroneid-core-c](https://github.com/opendroneid/opendroneid-core-c): message formats, Wi-Fi NAN and Beacon frame layout (`opendroneid.h`, `odid_wifi.h`, `wifi.c`)
- [opendroneid transmitter-linux](https://github.com/opendroneid/transmitter-linux): BLE frame layout
- [opendroneid receiver-android](https://github.com/opendroneid/receiver-android): BT4/BT5, NAN and Beacon reception
- [opendroneid wireshark-dissector](https://github.com/opendroneid/wireshark-dissector)
- [sxjack/uav_electronic_ids](https://github.com/sxjack/uav_electronic_ids): ESP32 transmitter and receiver
- [dump3411](https://github.com/ifnull/dump3411): Linux receiver, channel hopping defaults

Hardware and regulation:

- [ESP-IDF RF Coexistence, ESP32-C5](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c5/api-guides/coexist.html): Wi-Fi sniffer and BLE scan coexistence
- [CAAS AS-10 Broadcast Remote Identification](https://www.caas.gov.sg/docs/default-source/default-document-library/as-10_broadcast-remote-identification.pdf): transmit rates and methods per EN 4709-002
- [TBRD: TESLA Authenticated UAS Broadcast Remote ID](https://arxiv.org/pdf/2510.11343): Message Pack rate under F3411-22a
- [US 2023/0294848, Off-Channel UAV Remote ID Beaconing](https://patents.justia.com/patent/20230294848): Beacon operating and social channels
- [Drone Remote ID, btframework](https://www.btframework.com/droneremoteid.htm): BT5 Long Range sent together with BT4 Legacy (F3411-19)

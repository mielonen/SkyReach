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
| Per-drone state across packets | no, `odid_message_process_pack()` re-initialises its `ODID_UAS_Data` on every call | transport sessions and drone entities, see [State merge](#state-merge) |
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
  channel hopping                        OUI FA:0B:BC /         ─► state merge (session / entity)
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

The same drone sends from different MAC addresses over Wi-Fi and BLE, BLE addresses can rotate, and in BT4 Legacy mode Location packets carry no UAS ID. Data is therefore kept on three separate levels:

| Level | What it is | Key |
|---|---|---|
| Raw observation | one received frame, stored with its decoded content | receive time + source |
| Transport session | observations from one radio source over time | (broadcast method, MAC) |
| Drone entity | logical drone built from one or more transport sessions | set of (IDType, UASID) |

Rules between the levels:

1. Every observation belongs to exactly one transport session.
2. A transport session belongs to at most one drone entity at a time. An unattached session is a valid state.
3. Raw observations and their session IDs are the permanent record. Drone entities are a derived view and can be rebuilt from the log.

#### Transport sessions

- A session starts with the first frame from a (broadcast method, MAC) source.
- It ends after 30 s without frames, or when the source starts sending a different UAS ID. A new session is then started for the new identity.
- In BT4 Legacy mode the session links Location, System, etc. to the Basic ID sent from the same address. Message Packs (BT5, NAN, Beacon) carry Basic ID and Location in the same frame.
- Duplicates are dropped within the session by (message type, message counter). The counter is per transmitter and method, so the same content received over Wi-Fi and BLE is two observations, not a duplicate.
- Each session keeps its own position track.

#### Drone entities

- A session is attached to an entity when it has a valid Basic ID. An entity holds a set of keys, one per Basic ID type (up to five). Sessions are attached to the same entity when any key matches. Entities can merge late, for example when BLE first shows only the CAA registration and Wi-Fi only the serial number.
- A Basic ID is not used as a key if IDType is 0 or reserved (5–15), or if UASID is empty or all zeros. Serial numbers can also be checked against the ANSI/CTA-2063-A format.
- Operator ID and operator location are never keys. One operator can fly several drones.
- A session without a valid Basic ID stays unattached and is shown as an unidentified source. It is never attached to an entity by position or time. Proximity may be shown in the UI as a hint only.
- The entity position is chosen by Location timestamp across its sessions, not by receive time. An older position never replaces a newer one. Hour wrap and fallback rules are described under [Time](#time).

#### Conflicts

- A source switching to a different UAS ID within a short window is logged as an anomaly (spoofer, test tool or MAC collision). The session split keeps BT4 Location frames from being attached to the wrong drone.
- If sessions of one entity report positions that are physically impossible to reconcile (distance greater than max speed × time), positions are not averaged or overwritten. The session tracks stay separate and the entity is flagged with conflicting sources.
- Changes to static fields during a flight (Operator ID, UA type, System classification) are logged as events. The latest value is shown.

#### Lifetime

- An entity is stale when none of its sessions has received frames for 30 s. After 3 min the flight is closed.
- The same key seen again later is the same entity with a new flight.
- Session ID and UTM UUID (ID types 3 and 4) change per flight by design. No link between flights is inferred.

#### Known limits

- A drone with both a built-in Remote ID and an add-on module broadcasts two serial numbers and shows up as two entities. They cannot be merged reliably.
- An add-on module can be moved between aircraft, so a serial number identifies the transmitter, not the airframe.

#### Logging

Each raw observation is logged with its session ID. Session start and end, session-to-entity attachments, entity merges and anomalies are logged as separate events, so entities can be rebuilt offline with different rules.

#### Open items to measure

1. Whether BLE addresses stay fixed during a flight or rotate, and whether BT4 and BT5 advertising sets share an address.
2. Whether NAN and Beacon use the same MAC, and whether the Wi-Fi MAC changes at power-up.
3. How often Basic ID is sent relative to Location in BT4 Legacy mode. This sets how long a BT4 session stays unattached.

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

#### Two clocks

- Monotonic clock (`esp_timer`, µs): ordering of observations, session timeouts (30 s, 3 min) and duplicate windows. It never jumps.
- Wall clock (UTC): hour wrap of Location timestamps and human-readable logs. It can jump when it is set or corrected.

Timeouts and ordering never use the wall clock.

#### Receive time

Frames are timestamped on the C5 at reception (`rx_ctrl.timestamp` for Wi-Fi), not on the P4 after IPC, so buffering and transfer delay do not affect the receive time. The C5 timestamp is mapped to the P4 monotonic clock. The mapping is part of the IPC design.

#### Wall clock source

The MAX31329 RTC is kept in UTC and is the wall clock until GNSS is available. Clock quality is one of:

- `unset`: RTC never set or lost power
- `manual`: set by the user or over USB/serial
- `rtc`: running from an earlier set
- `gnss`: set from GNSS (later phase)

While the clock is `unset`, Location timestamps are not used for ordering and the hour wrap is not resolved; the monotonic receive time is used instead.

The RTC has ±5 ppm typical accuracy at 25 °C (about 0.4 s per day), more in the cold. This is enough for the hour wrap but not for latency measurement over weeks. In the GNSS phase the 1-PPS signal can drive the RTC CLKIN input. Sub-second time comes from the monotonic clock.

#### Drone timestamps

- Location timestamp: tenths of seconds after the full hour. The library only checks for `0xFFFF` when decoding (`decodeTimeStamp()`), so SkyReach treats values above 36000 as invalid. The raw value is still logged. The hour is taken from the wall clock, choosing the hour that puts the timestamp closest to the receive time (valid while the wall clock is within ±30 min of UTC).
- If the Location timestamp is invalid or missing, ordering falls back to the monotonic receive time. Equal timestamps from different methods are the same position, not a new point.
- System timestamp: seconds since 2019-01-01 UTC, only present from protocol version 2 (F3411-22). A value of 0 or `ProtoVersion` < 2 means no timestamp. The library does not check this. The transmitter clock can be wrong (for example before GNSS fix), so it is only used as a cross-check. The median over several drones can serve as a rough time source when the RTC is `unset`.
- Auth page 0 timestamp uses the same epoch and the same rules.

#### Latency flag

With a set wall clock, receive time minus Location timestamp gives the delay. A delay well above `TSAccuracy` plus clock error is flagged (stale position, replay or wrong transmitter clock). The flag does not reject the observation.

#### Logged time fields

Original time values are never replaced by a computed UTC time. Each value is logged as received, and any derived time is stored in a separate field.

Original values:

- receive time: raw C5 timestamp, P4 monotonic time (µs) and the wall clock reading at reception
- Location timestamp: raw 16-bit value (tenths of seconds after the hour, `0xFFFF` included)
- System and Auth page 0 timestamps: raw 32-bit value (seconds since 2019-01-01)

Derived values, each with the clock quality and the rule used:

- receive time in UTC
- Location time in UTC (hour resolved from the wall clock)
- System and Auth time in UTC
- latency (receive time minus Location time)

Clock set and correction events are logged with old time, new time and the monotonic instant. Derived times can then be recomputed offline without touching the original values.
- File timestamps on the SD card come from the RTC.
- GNSS phase: GPS time is converted to UTC (currently 18 leap seconds) before use.

#### Open items to measure

1. Offset and jitter between the C5 receive timestamp and the P4 clock over IPC.
2. RTC drift over the operating temperature range.
3. Share of real transmitters that send a System timestamp, and its accuracy.

### Logging

- Every received ODID frame is logged as raw hex alongside the decoded result, so parser bugs and new message types can be reprocessed later.
- Each record includes receive time (monotonic and UTC, see [Time](#time)), broadcast method, channel/PHY, RSSI, source MAC, message counter and `ProtoVersion`.
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

- [MAX31329 datasheet](https://www.analog.com/en/products/max31329.html): RTC accuracy and CLKIN input
- [ESP-IDF RF Coexistence, ESP32-C5](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c5/api-guides/coexist.html): Wi-Fi sniffer and BLE scan coexistence
- [CAAS AS-10 Broadcast Remote Identification](https://www.caas.gov.sg/docs/default-source/default-document-library/as-10_broadcast-remote-identification.pdf): transmit rates and methods per EN 4709-002
- [TBRD: TESLA Authenticated UAS Broadcast Remote ID](https://arxiv.org/pdf/2510.11343): Message Pack rate under F3411-22a
- [US 2023/0294848, Off-Channel UAV Remote ID Beaconing](https://patents.justia.com/patent/20230294848): Beacon operating and social channels
- [Drone Remote ID, btframework](https://www.btframework.com/droneremoteid.htm): BT5 Long Range sent together with BT4 Legacy (F3411-19)

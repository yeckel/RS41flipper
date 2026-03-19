# RS41 Tracker — Flipper Zero external app

Receives and decodes **Vaisala RS41** radiosondes using the Flipper Zero's
built-in CC1101 sub-GHz radio.  No external hardware required.

Developed by OK1CHP, inspired by [twatch-rs41](https://github.com/yeckel/twatch-rs41).

---

## Features

- Live decoding of RS41-SG / RS41-SGP radiosonde frames at 1 Hz
- Sonde serial number (e.g. `N1320638`)
- GPS position: latitude, longitude, altitude, satellite count
- Temperature with full sensor calibration (calframes 3–6)
- **Barometric pressure** with full sensor calibration (calframes 6–7) — RS41-SGP only
- RSSI display
- Frequency tunable from 400.1 to 406.0 MHz in 0.1 MHz steps

---

## Controls

| Button | Action |
|--------|--------|
| **OK** | Start / stop reception |
| **UP** | Increase frequency +0.1 MHz |
| **DOWN** | Decrease frequency −0.1 MHz |
| **BACK** | Exit app |

---

## Display

```
RS41  405.1 MHz [RX]
────────────────────
ID:N1320638  -84dBm
14.6685N  037.0377W
Alt: 1460m T:+19.0C
P: 849.1hPa Sats: 0
```

Fields show `---` while waiting for calibration frames or GPS fix.

---

## Radio configuration (CC1101)

| Parameter | Value |
|-----------|-------|
| Modulation | 2-FSK |
| Data rate | 4800 bps |
| Frequency deviation | ≈ 2.4 kHz |
| Channel filter BW | ≈ 58 kHz |
| Sync word | `0x086D` (bit-reversed RS41 preamble) |
| Frequency range | 400.1 – 406.0 MHz |
| Packet mode | Infinite length (manual FIFO drain via SPI) |

The CC1101 outputs bits MSB-first; RS41 transmits LSB-first.
Each FIFO byte is bit-reversed, then XOR-descrambled with a 64-byte key.

> **Note:** The Flipper firmware's `furi_hal_subghz_read_packet()` silently
> consumes the first FIFO byte as a length field, corrupting 318-byte frames.
> This app bypasses it with direct SPI FIFO reads.

---

## RS41 frame decoding

Each frame is 318 bytes in the FIFO (6 sync + 312 payload):

1. **Bit-reversal** — MSB↔LSB swap on every byte
2. **XOR descramble** — 64-byte repeating key, offset 8
3. **Skip** bytes 0–47 (Reed-Solomon parity, not yet used) and byte 48 (frame type)
4. **TLV block parsing** from byte 49, each validated with CRC-16/CCITT-FALSE:

| Block ID | Name   | Decoded fields |
|----------|--------|----------------|
| `0x79`   | STATUS | Serial number; rotating calibration data (calframes 0–50) |
| `0x7A`   | PTU    | Temperature and pressure ADC measurements (uint24 LE × 3) |
| `0x7B`   | GPS    | ECEF position (int32 × 0.01 m) → WGS-84 via Bowring iteration |

### Temperature calibration (calframes 3–6)

The sonde embeds calibration constants in the STATUS block, 16 bytes per frame,
rotating through 51 calframe indices.  Four consecutive calframes (3, 4, 5, 6)
are needed to assemble `Rf1`, `Rf2`, `co1[3]`, `calT1[3]`.

Formula (`rs1729` / `rs41ptu.c` algorithm):
```
g  = (f2 - f1) / (Rf2 - Rf1)
Rb = (f1·Rf2 - f2·Rf1) / (f2 - f1)
Rc = meas[0] / g - Rb
R  = Rc · calT1[0]
T  = (co1[0] + co1[1]·R + co1[2]·R²  + calT1[1]) · (1 + calT1[2])
```

### Pressure calibration (calframes 6–7, RS41-SGP only)

Same resistive-bridge structure as temperature, using `Fp1`, `Fp2`, `Cp[3]`:
```
g  = (fp2 - fp1) / (Fp2 - Fp1)
Rb = (fp1·Fp2 - fp2·Fp1) / (fp2 - fp1)
Rc = meas[3] / g - Rb
P  = Cp[0] + Cp[1]·Rc + Cp[2]·Rc²     (hPa)
```

---

## Build & deploy

Requires [ufbt](https://github.com/flipperdevices/flipperzero-ufbt):

```bash
pip install ufbt          # first time only
cd rs41_tracker
ufbt build                # compile
ufbt launch               # compile + install via USB
ufbt cli                  # open serial console
log                       # (inside ufbt cli) stream debug log
```

Tested on firmware **1.4.3 (release)**, API 87.1, Target 7, device on `/dev/ttyACM1`.

---

## Finding RS41 frequencies

Active sondes near you:

- <https://radiosondy.info>
- <https://sondehub.org>
- Typical Central-European range: **403–406 MHz**

---

## License

GNU GPL v3 — see top-level LICENSE file.

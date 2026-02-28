# 1602A Sniffer

Sniff a 1602A style LCD interface from a Geiger counter, mirror the screen text, and turn it into real data.

This project is a demonstrator: take a device that only speaks via an HD44780 style LCD header and make it stream useful numbers. Think Fallout, Pip Boy vibes, but real sensor data.

Target use case:
A common handheld Geiger counter that drives a 16x2 LCD and does not provide serial output. Example listing:
https://www.ebay.com/itm/357934090548

What it does:
- Passive sniff of the LCD bus, no LCD required
- Mirrors the 16x2 text over USB serial
- Extracts values from display labels
  - R: rate in uSv per hour
  - A: average in uSv per hour
- Computes derived metrics
  - Peak rate
  - Running dose in uSv
  - EMA smoothing, 1 minute and 10 minute
  - Approx rad per hour and rad per second
- Alarm states for demos
  - FOUND, sustained rise above baseline
  - FLIGHT, sustained elevated band
  - DANGER, absolute threshold
  - UNHEALTHY, absolute threshold

Why this exists:
- This shows how far you can go without vendor firmware, without UART pins, without reverse engineering the MCU.
- It is a clean stepping stone to ESP32, Meshtastic, and a web UI.

## Hardware

This project now supports two practical wiring targets:

- Arduino Uno as the original 5V reference build
- Arduino Micro as the working PlatformIO port used for the traced Geiger harness

Connections are documented in:
docs/WIRING.md

Working Arduino Micro mapping used by the current PlatformIO firmware:

- RS -> D2
- RW -> D4
- E -> D12
- D0 -> D11
- D1 -> D10
- D2 -> D9
- D3 -> D8
- D4 -> D7
- D5 -> D6
- D6 -> D5
- D7 -> D3

Notes:

- This matches the traced harness where the Arduino-side D3 and D4 wires are swapped.
- On this Micro wiring, the byte bus is captured directly and the firmware infers command vs data from the HD44780 byte stream, because the traced control-line levels are not as clean as the byte data.

## Output

Example lines:
- LCD|R:00.16 usv/h  1|A:00.18 usv/h  o|
- rate_usvph=0.16 avg_usvph=0.18 ema1m_usvph=0.15 ema10m_usvph=0.12 peak_usvph=0.26 dose_uSv=0.004 rate_radph=0.00001600 rate_rads=0.0000000044 dose_rad=0.00000040 alarm=OK

More examples in:
docs/SERIAL_EXAMPLES.md

## Units and rad note

The device appears to print uSv per hour. The rad conversions here are a demonstration only.

uSv is equivalent dose. rad is absorbed dose. Converting between them depends on radiation type and quality factor. This code assumes gamma or x ray like conditions where quality factor is near 1, so:
- 1 Sv is treated as 1 Gy
- 1 Gy equals 100 rad

If you want strict correctness across radiation types, keep everything in Sv units or calibrate for your tube and spectrum.

## Next steps

- Port to ESP32 S3 and add WiFi, web UI, logging
- Add Meshtastic telemetry packets
- Add physical UI, servo needle gauge, buttons, RGBW indicator

## Topics to use on GitHub

Suggested repo topics:
1602a, hd44780, lcd, sniffer, reverseengineering, geigercounter, radiation, doserate, esp32, meshtastic, fallout, pipboy, retrocomputing, iot

## Repo layout

- `arduino/1602A_Sniffer/` – Arduino Uno sketch for the original reference wiring.
- `docs/` – Wiring and serial-output examples you can read before wiring up the counter.
- `PlatformIO` scaffolding (`platformio.ini`, `src/main.cpp`, `include/`, `lib/`, `test/`) now includes the working Arduino Micro implementation.

## Building locally

1. For the original Uno path, open `arduino/1602A_Sniffer/1602A_Sniffer.ino` in the Arduino IDE and wire the board per `docs/WIRING.md`.
2. For the working Micro path, open the repo in PlatformIO and build the default `micro` environment.
3. If your upload port is not auto-detected, pass it explicitly, for example `pio run -e micro -t upload --upload-port COM20`.
4. Open a 115200 baud monitor and watch the serial stream mirror the display and report metrics; the sniffer is passive so no LCD is connected.

## License

MIT

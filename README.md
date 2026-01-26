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

This sketch is written for Arduino Uno as a safe first step with 5V logic.

Connections are documented in:
docs/WIRING.md

## Output

Example lines:
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

## License

MIT

# Wiring

Goal:
Sniff the LCD bus coming from the Geiger counter main board. No LCD needs to be connected.

This assumes the counter exposes the standard 1602A 16 pin header signals:
VSS, VDD, VO, RS, RW, E, D0, D1, D2, D3, D4, D5, D6, D7, A, K

Only these matter for sniffing:
RS, RW, E, D0 to D7, and GND reference.

## Power and safety

- Use a common ground between Arduino and the counter board.
- Do not power the counter from the Arduino 5V pin unless you know the current draw is safe.
- Best practice: power the counter with its normal supply, then tie grounds.

## Arduino Uno pin mapping used by the sketch

Counter signal -> Arduino pin
- E  -> D2
- RS -> D4
- RW -> D5
- D0 -> A0
- D1 -> A1
- D2 -> A2
- D3 -> A3
- D4 -> D6
- D5 -> D7
- D6 -> D8
- D7 -> D9
- GND -> GND

Do not connect VDD, VO, A, K to the Arduino. Not needed for sniffing.

## Header pin order reminder

A typical 1602A header order is:
1 VSS
2 VDD
3 VO
4 RS
5 RW
6 E
7 D0
8 D1
9 D2
10 D3
11 D4
12 D5
13 D6
14 D7
15 A
16 K

If your counter board is unmarked, use continuity to locate ground first, then find the E line by looking for the fast pulsing signal during screen updates.

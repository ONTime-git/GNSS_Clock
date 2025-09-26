# GNSS Clock
**GNSS Clock** is an autonomous, highly accurate clock designed to provide the exact time anywhere in the world. It uses GNSS signals to synchronize automatically and adapts to local time zones, daylight saving time (DST), and national time policies depending on your location.

## Features
- **Global synchronization:** Works anywhere GNSS signals are available.
- **Automatic local adjustment:** Adapts to local time rules and DST.
- **Highly accurate:** Maintains precise time under all conditions.
- **Reliable:** Independent operation, even in remote locations.
- **Embedded-friendly:** Designed for STM32U0 microcontrollers.

## Hardware
- **Microcontroller:** STM32U0 series – ultra-low-power, ideal for continuous operation.
- **GNSS Module:** u-blox DAN-F10N – biband L1/L5, supports multiple GNSS constellations for reliable time fixes.

## Software
- **GNSS Communication:** Inspired by **ubxlib** – some source code was adapted and heavily modified for this project.
- **Time Zone Management:** Uses **tzdata** to map GNSS coordinates to local time zones and apply DST rules automatically.
- **Embedded Firmware:** Written in C for STM32U0, optimized for low-power operation.

## How It Works
1. The GNSS module receives satellite signals and computes precise UTC time.
2. The microcontroller processes the data to extract accurate time information.
3. Coordinates are mapped to the local time zone using tzdata.
4. DST rules and local policies are applied automatically.
5. The clock updates its display or output in real time, remaining accurate under all conditions.

## Use Cases
- Accurate wall clocks or desk clocks anywhere in the world.
- Embedded systems requiring precise timekeeping (IoT, logging devices, industrial controllers).
- Remote installations where internet-based NTP is not available.

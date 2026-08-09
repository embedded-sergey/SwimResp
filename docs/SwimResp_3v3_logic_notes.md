# SwimResp 3.3 V‑logic notes  
SwimResp can operate with 3.3 V‑logic Nano‑family boards (Nano 33 BLE, Nano 33 IoT, RP2040, Nano ESP32), which may provide additional functionality.

This topic is outside the scope of the main build guide. Several design and assembly adjustments are required, and the list below reflects unverified considerations only.

1. All SwimResp components (status LED, push button, L298N motor driver board, OLED display, Pico‑O2 OEM logger) operate correctly at 3.3 V logic level.

2. Power the OLED display from the 3.3 V pin of a 3.3 V‑logic Arduino Nano to ensure reliable two‑way I²C communication.
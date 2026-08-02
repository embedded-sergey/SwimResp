---
title: "SwimResp Build Guide"
author: "Sergey Morozov"
date: "14 July 2026"
revision: "0.1"
toc: true
toc-depth: 3
---

# SwimResp Build Guide
**Revision:** 0.1
**Author:** Sergey Morozov  
**Date:** 13 July 2026

<!-- begin-md-image -->
![SwimResp Device](images/SwimResp_photo.jpg)
<!-- end-md-image -->

# Table of Contents

1. [Introduction](#1-introduction)
2. [Safety Notes](#2-safety-notes)  
   2.1 [Environmental Safety](#21-environmental-safety)  
   2.2 [Electrical Safety](#22-electrical-safety)  
   2.3 [Mechanical and Tool Safety](#23-mechanical-and-tool-safety)
3. [Bill of Materials](#3-bill-of-materials)
4. [Tools and Consumables](#4-tools-and-consumables)

# 3. Bill of Materials

Price estimates for generic components use Amazon.com  as a reference, based on pricing available in August 2026. Amazon is chosen because it provides broad international availability, relatively stable pricing, and consistent product listings, which makes it a practical baseline for reproducible cost estimates. Because many components on Amazon are sold only in bulk packs, the bill of materials lists the price of the entire pack even when only one piece is needed, which inflates the apparent cost of a single device and results in unnecessary surplus. In practice, individual components can often be purchased at lower cost from suppliers that support small‑quantity or single‑unit purchasing, including local electronics stores, online marketplaces, and global engineering distributors such as Mouser or Digi‑Key. Prices for specialized sensors and measurement devices are taken directly from manufacturers’ websites when these items are not consistently available through the above distributors.

Prices listed in this BOM exclude VAT, customs duties, and delivery costs.

| Part Name | Manufacturer | Quantity | Unit Price (USD)¹ | Total Price (USD) | Notes |
|-----------|--------------|----------|--------------------|--------------------|-------|
| Industrial ABS enclosure, IP65, 200×120×75 mm | Generic | 1 pc | 10 | 10 | – |
| Waterproof automotive connectors (Superseal 1.5‑style), 2‑pin | Generic | 2 male–female pairs | 8 | 8 | Bulk pack (3–5 pairs). Female connectors are also needed for a pump and a motor connected to SwimResp²|
| L298N Motor Driver Board | Generic | 1 pc | 5 |  5 | – |
| Red and black wires, 20 AWG | Generic | 1 m each | 3 | 3 | Use 16 AWG for high‑current pumps and motors |
| Round rocker switch, mounting hole diameter 15 mm, 2-pin | Generic | 1 pc | 6 | 6 | Bulk pack (3–5 pcs) |
| DC power jack, steel, 5.5×2.1 mm | Generic | 1 pc | 7 | 7 | Bulk pack (3–5 pcs) |
| Arduino Nano–compatible board (ATmega328P) with Mini-B USB cable and screw‑terminal adapter | Generic | 1 kit | 9 | 9 | The board can be replaced by the Arduino Nano rev.3 or 3.3 V-board from Nano family² |
| Push button, mounting hole diameter 16 mm, 2-pin | Generic | 3 pcs | 10 | 10 | Bulk pack (10-24 pcs) |
| LED, 5 mm | Generic | 1 pc | 6 | 6 | Bulk pack (50–200 pcs) |
| Resistor, 220 Ω, ¼ W | Generic | 1 pc | 6 | 6 | Bulk pack (20–200 pcs) |
| OLED 128x64 I²C Monochrome Display, 5 V | Generic | 1 pc | 9 | 9 | – |
| Solderless breadboard BB170, 47x35x8.5 mm | Generic | 1 pc | 6 | 6 | Bulk pack (3–6 pcs) |
| Female to male jumper wires, square jumpers, 10 cm | Generic | 1 kit | 5 | 5 | – |
| M3 Hex Spacers Standoff Kit | Generic | 1 kit | 12 | 12 | To attach terminal and motor driver boards to enclosure |   
| Optical oxygen meter Pico‑02 OEM module, UART, 3.3–5 V | PyroScience | 1 pc | NA⁴ | NA⁴ | Compatible with optical REDFLASH oxygen sensors and PT1000 |

**Grand total: 102 USD excluding price for Pico oxygen meter⁴ **

*¹ Unit prices reflect the smallest purchasable quantity on Amazon.com. When only bulk packs are available, the listed price corresponds to the full pack even if a single piece is required.*

*² This BOM does not include pumps or their cables.*

*³ The design is compatible with 3.3 V‑logic Arduino Nano–family boards (e.g., Nano Every, Nano 33 IoT, Nano 33 BLE) provided that the 5 V relay module is replaced with a 3 V/3.3 V logic‑compatible relay module powered from 3–3.3 V. All other digital I/O (status LED, push button, PWM control signals) operate correctly at 3.3 V logic levels.*

*⁴ Price available only via manufacturer quotation.*

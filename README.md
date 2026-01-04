Hermes-Lite 2.x (DSP/Gateware Fork)
===============================

Dieses Repository ist ein Fork/Arbeitsstand der Hermes‑Lite 2 Gateware/DSP.  
Allgemeine Projektinformationen finden sich auf der [Hermes-Lite Web Page](http://www.hermeslite.com) und im Hermes/OpenHPSDR Umfeld.

## Envelope Shaping (CW Key‑Click Reduktion)

In `gateware/rtl/radio_openhpsdr1/radio.v` wird die CW‑TX‑Amplitude nicht hart ein/aus geschaltet, sondern über eine Rampe (`tx_cwlevel`) geformt. Diese Version erweitert das bestehende CW‑Shaping um konfigurierbare Rise/Fall Zeiten sowie eine Maximalamplitude.

### Parameter (FPGA)

Die Parameter werden über die bestehende OpenHPSDR/HL2 Command‑Schnittstelle (6‑bit `cmd_addr`, 32‑bit `cmd_data`) in das `radio`‑Modul geschrieben.

- `cmd_addr = 0x18` (`ENV_CFG0`):
  - `cmd_data[15:0]`  = `env_rise_us` (µs, 0 = sofort)
  - `cmd_data[31:16]` = `env_fall_us` (µs, 0 = sofort)
- `cmd_addr = 0x19` (`ENV_CFG1`):
  - `cmd_data[15:0]`  = `env_max_amp_q15` (Q1.15, `0x7FFF` = 1.0)
  - `cmd_data[31:16]` = reserviert

Defaults beim Reset:
- `env_rise_us = 3000`
- `env_fall_us = 3000`
- `env_max_amp_q15 = 0x7FFF`

### Implementationshinweis

Die Rampe nutzt eine Bresenham‑artige Fehlerakkumulation (bis zu 2 Schritte pro FPGA‑Takt), um auch Rampen schneller als 1 LSB/Clock abbilden zu können (z. B. 3 ms bei 76.8 MHz). 

## Bezug zu SOTA‑CW

Das Projekt [Hermes_Lite_SDR_SOTA_CW](https://github.com/philibertschlutzki/Hermes_Lite_SDR_SOTA_CW) nutzt/erwartet eine solche TX‑Envelope‑Steuerung zur Key‑Click‑Reduktion und dokumentiert die Backend‑Seite.

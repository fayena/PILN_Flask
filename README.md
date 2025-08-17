# PiLN Flask – Raspberry Pi Kiln Controller

> **UPDATE (2025-08-16):** Major refresh. The app now runs on **Flask + Gunicorn**, fronted by **nginx**, with a cleaner UI, faster charts, and a proper JSON API.  
> ⚠️ Electricity and heat are dangerous. Evaluate risk and make your own go/no-go decision. You are responsible for your setup and safety.

---

## Table of Contents

- [What’s New](#whats-new)
- [Safety Disclaimer](#safety-disclaimer)
- [Hardware](#hardware)
- [Quick Start](#quick-start)
- [Directory Layout](#directory-layout)
- [Database](#database)
- [API](#api)
  - [/api/health](#get-apihealth)
  - [/api/status](#get-apistatus)
  - [/api/data](#post-apidata)
- [UI Routes](#ui-routes)
- [nginx Reference](#nginx-reference)
- [Firing Daemon Notes](#firing-daemon-notes)
- [Troubleshooting](#troubleshooting)
- [Screenshots](#screenshots)
- [Credits](#credits)
- [Appendix: Useful Commands](#appendix-useful-commands)

---

## What’s New

- ✅ Switched from CGI scripts to **Flask** (`/home/pi/PILN/api/app.py`) served by **Gunicorn**
- ✅ **nginx** reverse proxy (simpler & more reliable for this project)
- ✅ **DB-driven status**: `/api/status` pulls from SQLite (`profiles`, `firing`, `segments`) and checks the `pilnfired.py` daemon
- ✅ **Incremental charting**: `/api/data` returns only new points after a timestamp so charts stay fast
- ✅ Cleaned up **templates** and CSS; consistent actions (links vs buttons) and mobile-friendly header with logo + live status
- ✅ Old files no longer used: `app/check.cgi`, `app/data.cgi`, `app/data.json` (safe to remove)

> Hardware, wiring, and DB schema remain compatible with the legacy projects.

---

## Safety Disclaimer

Electric kilns draw serious current and reach extreme temperatures. Use properly rated wiring, relays/SSRs, and connectors. Poor electrical work can cause injury, death, or fire. **Monitor every firing.** A kiln sitter/cone and independent safety mechanisms are strongly recommended.

---

## Hardware

- Raspberry Pi (Pi 4 used here; others work)
- MAX31856 thermocouple amplifier (SPI)
- Type-K thermocouple + high-temp thermocouple wire
- Two high-current relays/SSRs (rated for your load), 12 V coil supply or SSR control
- ULN2803A / driver (if driving relays), fan, DIN rails, enclosure, terminals, etc.

> Use parts correctly rated for amperage and temperature.

---

## Quick Start

1. **Enable SPI/I²C** in `raspi-config` (Interfacing Options).
2. **Run the setup script** on Raspberry Pi OS:

   ```bash
   cd ~
   wget https://raw.githubusercontent.com/fayena/PILN_Flask/refs/heads/main/pilnsetup.sh
   chmod +x pilnsetup.sh
   ./pilnsetup.sh


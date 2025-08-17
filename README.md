
````markdown
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
````

3. **Services** (installed by the script):

   ```bash
   # status
   systemctl status pilnapi.service
   systemctl status pilnfired.service

   # restart if needed
   sudo systemctl restart pilnapi.service
   sudo systemctl restart pilnfired.service
   ```

4. **Open the UI** in your browser:

   ```
   http://<your-pi-ip>/
   ```

---

## Directory Layout

```
/home/pi/PILN
  ├── api/app.py           # Flask app (API + UI routes)
  ├── daemon/pilnfired.py  # Firing daemon (writes to SQLite)
  ├── db/PiLN.sqlite3      # Database
  ├── style/               # CSS, JS (Chart.js, DataTables, jQuery)
  ├── images/              # Logos, screenshots
  └── template/            # Jinja templates (header, home, view, chart, etc.)
```

---

## Database

Tables (unchanged):

* `profiles(run_id, state, notes, p_param, i_param, d_param, start_time, end_time, …)`
* `segments(run_id, segment, set_temp, rate, hold_min, int_sec, start_time, end_time)`
* `firing(run_id, segment, dt, set_temp, temp, int_temp, pid_output)`

**Recommended index (faster charts):**

```sql
CREATE INDEX IF NOT EXISTS idx_firing_run_dt ON firing(run_id, dt);
```

---

## API

### `GET /api/health`

Returns basic liveness.

```json
{ "ok": true }
```

### `GET /api/status`

Returns daemon state + most recent kiln status derived from DB.

```json
{
  "daemon": { "running": true },
  "status": "active",           // "active" | "idle" | "down"
  "run_profile": "219",         // "none" if idle
  "run_segment": "1",           // "n/a" if idle/down
  "readtemp": "140",
  "ramptemp": "138",
  "targettemp": "121",
  "segtime": "0:00:00"
}
```

* `status = "active"` when `profiles.state = 'Running'` (case-insensitive) and samples exist
* `status = "idle"` when no active run
* `status = "down"` if the `pilnfired.py` process isn’t found

### `POST /api/data`

**Body:** `{"run_id":"212","since":"YYYY-MM-DD HH:MM:SS"}`
`since` optional: omit or `null` for full series; set to a timestamp for deltas.

**Response:**

```json
{
  "dt_iso": ["2025-08-16 10:00:00", "..."],
  "dt_label": ["10:00 AM", "..."],
  "segment": [1, 1, 2, "..."],
  "set_temp": [200, 201, 300, "..."],
  "temp": [197, 199, 298, "..."],
  "pid_output": [42, 60, 55, "..."],
  "last_iso": "2025-08-16 11:23:45"
}
```

**Curl tests:**

```bash
# status
curl -s http://127.0.0.1:5000/api/status | python3 -m json.tool | head

# full series for run 212
curl -s -X POST http://127.0.0.1:5000/api/data \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"212","since":null}' | python3 -m json.tool | head

# incremental after a timestamp
curl -s -X POST http://127.0.0.1:5000/api/data \
  -H 'Content-Type: application/json' \
  -d '{"run_id":"212","since":"2025-08-16 10:30:00"}' | python3 -m json.tool | head
```

---

## UI Routes

Read-only (GET):

* `/` or `/home` – profile list (DataTables)
* `/view?run_id=<id>&state=<State>&notes=...` – profile detail; appends chart when `Running/Stopped/Completed`
* `/new` – create profile form
* `/editcopy?run_id=<id>` – edit/copy profile form
* `/del_conf?run_id=<id>&state=...` – delete confirmation

Mutations (POST):

* `/run`, `/savenew`, `/saveupd`, `/delete`, `/stop`, `/notes_save`

> Navigation is consistent; destructive/mutating actions remain POST.

---

## nginx Reference

The installer drops a working config. For reference:

```nginx
server {
  listen 80;
  server_name _;

  # Static assets served directly
  location /style/  { root /home/pi/PILN; }
  location /images/ { root /home/pi/PILN; }

  # Everything else proxied to Flask/Gunicorn
  location / {
    proxy_pass         http://127.0.0.1:5000;
    proxy_set_header   Host              $host;
    proxy_set_header   X-Real-IP         $remote_addr;
    proxy_set_header   X-Forwarded-For   $proxy_add_x_forwarded_for;
    proxy_set_header   X-Forwarded-Proto $scheme;
  }
}
```

---

## Firing Daemon Notes

* Uses PID gains from `profiles` (`p_param`, `i_param`, `d_param`)
* Optional Åström–Hägglund **autotune** (off by default; enable in code if desired)
* Removed “elements off between segments” to avoid dips at boundaries
* Seeds next segment’s ramp from previous **ramp target** (not PV) to avoid snap at changeover

```bash
sudo systemctl restart pilnfired.service
sudo systemctl status  pilnfired.service
```

---

## Troubleshooting

* **404 CSS/JS**: confirm nginx `location /style/` and `/images/` root → `/home/pi/PILN`
* **500 on `/api/data`**: check JSON body, ensure rows exist for that `run_id`
* **Status stuck `idle`**: verify `profiles.state` is `Running` and the daemon is inserting rows
* **Status `down`**: `systemctl status pilnfired.service` and review logs
* **Charts slow**: create the `idx_firing_run_dt` index

---

## Screenshots

Add your images and update the paths:

```
![Home](images/screens/home.png)
![Profile](images/screens/profile.png)
![Chart](images/screens/chart.png)
```

---

## Credits

This project builds on:

* [https://github.com/pvarney/PiLN](https://github.com/pvarney/PiLN)
* [https://github.com/BlakeCLewis/PILN](https://github.com/BlakeCLewis/PILN)

Thanks to the original authors and contributors.

---

## Appendix: Useful Commands

```bash
# API sanity checks
curl -s http://127.0.0.1:5000/api/health
curl -s http://127.0.0.1:5000/api/status | python3 -m json.tool | head

# Tail services (journald)
journalctl -u pilnapi.service -f
journalctl -u pilnfired.service -f

# SQLite peek
sqlite3 /home/pi/PILN/db/PiLN.sqlite3 \
  "SELECT run_id, segment, dt, set_temp, temp FROM firing ORDER BY dt DESC LIMIT 5;"
```



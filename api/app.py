# /home/pi/PILN/api/app.py
from flask import Flask, jsonify, request, g
import json
import os
import sqlite3
from datetime import datetime

import subprocess
try:
    import psutil
except Exception:
    psutil = None

APP_DIR = "/home/pi/PILN"
STAT_FILE = os.path.join(APP_DIR, "app", "pilnstat.json")
DB_PATH = os.path.join(APP_DIR, "db", "PiLN.sqlite3")

app = Flask(__name__)

# -------- helpers --------
def get_db():
    if "db" not in g:
        g.db = sqlite3.connect(DB_PATH, detect_types=sqlite3.PARSE_DECLTYPES, check_same_thread=False)
        g.db.row_factory = sqlite3.Row
    return g.db

@app.teardown_request
def close_db(exc):
    db = g.pop("db", None)
    if db:
        db.close()

def fmt_label(iso_s: str) -> str:
    # iso_s expected like "YYYY-MM-DD HH:MM:SS"
    dt = datetime.strptime(iso_s.split(".")[0].replace("T", " "), "%Y-%m-%d %H:%M:%S")
    return dt.strftime("%I:%M %p").lstrip("0")

# -------- existing routes --------
@app.route("/api/health")
def health():
    return jsonify({"ok": True})

@app.route("/api/status")
def status():
    resp = {"daemon": {"running": daemon_running()}}
    try:
        with open(STAT_FILE) as f:
            data = json.load(f)
        resp.update(data)  # preserves existing top-level keys (run_profile, readtemp, etc.)
        return jsonify(resp)
    except FileNotFoundError:
        resp["error"] = "status file not found"
        # Keep your original 404 for compatibility
        return jsonify(resp), 404
# -------- NEW: incremental data endpoint --------
@app.post("/api/data")
def data_delta():
    """
    Body: {"run_id": "ABC123", "since": "YYYY-MM-DD HH:MM:SS"}  # since is optional
    Returns new points after 'since' (or full series if since is omitted).
    """
    body = request.get_json(silent=True) or {}
    run_id = str(body.get("run_id", "")).strip()
    since = body.get("since")  # None or ISO "YYYY-MM-DD HH:MM:SS"

    if not run_id:
        return jsonify({"error": "run_id required"}), 400

    # Optional: cap initial size by adding e.g. "LIMIT 5000" on first load
    if since:
        sql = """SELECT dt, segment, set_temp, temp, pid_output
                 FROM firing
                 WHERE run_id = ? AND dt > ?
                 ORDER BY dt ASC"""
        params = (run_id, since)
    else:
        sql = """SELECT dt, segment, set_temp, temp, pid_output
                 FROM firing
                 WHERE run_id = ?
                 ORDER BY dt ASC"""
        params = (run_id,)

    db = get_db()
    rows = db.execute(sql, params).fetchall()

    dt_iso, dt_label, seg, set_t, temp, pid = [], [], [], [], [], []
    for r in rows:
        iso = str(r["dt"])
        dt_iso.append(iso)
        dt_label.append(fmt_label(iso))
        seg.append(r["segment"])
        set_t.append(r["set_temp"])
        temp.append(r["temp"])
        pid.append(r["pid_output"])

    return jsonify({
        "dt_iso": dt_iso,
        "dt_label": dt_label,
        "segment": seg,
        "set_temp": set_t,
        "temp": temp,
        "pid_output": pid,
        "last_iso": dt_iso[-1] if dt_iso else since
    })
def daemon_running() -> bool:
    target = "pilnfired.py"
    if psutil:
        for p in psutil.process_iter(["cmdline"]):
            cl = p.info.get("cmdline") or []
            if any(target in s for s in cl):
                return True
        return False
    # fallback if psutil isn't available in this env
    return subprocess.run(
        ["pgrep", "-f", target],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    ).returncode == 0

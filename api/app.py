# /home/pi/PILN/api/app.py
from flask import Flask, jsonify, request, g
import json
import os
import sqlite3
from datetime import datetime, timedelta

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
    try:
        running = daemon_running()  # you already have this function
        payload = status_from_db()
        if not running:
            payload["status"] = "down"
            payload["run_profile"] = "none"
            payload["run_segment"] = "n/a"
        resp = jsonify({"daemon": {"running": running}, **payload})
        resp.headers["Cache-Control"] = "no-store, max-age=0"
        return resp, 200
    except Exception as e:
        app.logger.exception("GET /api/status failed")
        return jsonify({
            "daemon": {"running": False},
            "run_profile": "none",
            "run_segment": "n/a",
            "status": "down",
            "readtemp": "n/a",
            "ramptemp": "n/a",
            "targettemp": "n/a",
            "segtime": "0:00:00",
            "error": str(e)
        }), 200


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
    
def _normalize_iso(s):
    if s is None: return ""
    s = str(s).strip().replace("T", " ")
    if "." in s: s = s.split(".", 1)[0]
    return s

def _parse_iso(s):
    return datetime.strptime(_normalize_iso(s), "%Y-%m-%d %H:%M:%S")

def _fmt_hhmmss(delta: timedelta) -> str:
    total = int(delta.total_seconds())
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    return f"{h}:{m:02d}:{s:02d}"

def _get_running_run_id(db):
    """
    Prefer the 'running' profile whose firing has the most recent sample.
    Avoids relying on profiles.start_time (may not exist).
    """
    try:
        row = db.execute(
            """
            SELECT p.run_id
              FROM profiles p
             WHERE LOWER(p.state) = 'running'
             ORDER BY COALESCE( (SELECT MAX(f.dt) FROM firing f WHERE f.run_id = p.run_id), 0 ) DESC
             LIMIT 1
            """
        ).fetchone()
        return str(row["run_id"]) if row and row["run_id"] is not None else None
    except sqlite3.OperationalError:
        # profiles table might not exist yet
        return None


def status_from_db():
    try:
        db = get_db()

        running_run_id = _get_running_run_id(db)

        def latest_for(run_id):
            return db.execute(
                """SELECT run_id, dt, segment, set_temp, temp
                     FROM firing
                    WHERE run_id = ?
                    ORDER BY dt DESC
                    LIMIT 1""",
                (run_id,)
            ).fetchone()

        def last_overall():
            return db.execute(
                """SELECT run_id, dt, segment, set_temp, temp
                     FROM firing
                    ORDER BY dt DESC
                    LIMIT 1"""
            ).fetchone()

        def target_temp(run_id, segment):
            try:
                r = db.execute(
                    """SELECT set_temp FROM segments
                       WHERE run_id = ? AND segment = ?
                       LIMIT 1""",
                    (run_id, segment)
                ).fetchone()
                return None if not r or r["set_temp"] is None else float(r["set_temp"])
            except sqlite3.OperationalError:
                return None

        # ACTIVE (profiles says Running)
        if running_run_id:
            last = latest_for(running_run_id)
            if not last:
                return {
                    "run_profile": running_run_id,
                    "run_segment": "n/a",
                    "status": "active",
                    "readtemp": "n/a",
                    "ramptemp": "n/a",
                    "targettemp": "n/a",
                    "segtime": "0:00:00",
                }

            run_id  = str(last["run_id"])
            seg     = int(last["segment"]) if last["segment"] is not None else 0
            temp    = float(last["temp"]) if last["temp"] is not None else None
            set_tmp = float(last["set_temp"]) if last["set_temp"] is not None else None

            # segtime
            segtime = "0:00:00"
            try:
                last_dt = _parse_iso(last["dt"])
                r = db.execute(
                    """SELECT MIN(dt) AS dt FROM firing
                       WHERE run_id = ? AND segment = ?""",
                    (run_id, seg)
                ).fetchone()
                if r and r["dt"]:
                    seg_start = _parse_iso(r["dt"])
                    segtime = _fmt_hhmmss(max(timedelta(0), last_dt - seg_start))
            except Exception:
                pass

            # ramp °C/min over last few points
            ramptemp = "n/a"
            try:
                pts = db.execute(
                    """SELECT dt, temp FROM firing
                       WHERE run_id = ?
                       ORDER BY dt DESC LIMIT 6""",
                    (run_id,)
                ).fetchall()
                if len(pts) >= 2:
                    t0 = _parse_iso(pts[0]["dt"])
                    tn = _parse_iso(pts[-1]["dt"])
                    mins = (t0 - tn).total_seconds() / 60.0
                    if mins > 0:
                        ramptemp = f"{(float(pts[0]['temp']) - float(pts[-1]['temp']))/mins:.1f}"
            except Exception:
                pass

            tgt = target_temp(run_id, seg)

            return {
                "run_profile": run_id,
                "run_segment": str(seg) if seg else "0",
                "status": "active",
                "readtemp":   f"{temp:.0f}"     if temp    is not None else "n/a",
                "ramptemp":   f"{set_tmp:.0f}"  if set_tmp is not None else "n/a",  # same meaning as before
                "targettemp": f"{tgt:.0f}"      if tgt     is not None else "n/a",
                "segtime": segtime,
            }

        # IDLE — show last measured temp if any
        last_any = last_overall()
        if last_any and last_any["temp"] is not None:
            return {
                "run_profile": "none",
                "run_segment": "n/a",
                "status": "idle",
                "readtemp":   f"{float(last_any['temp']):.0f}",
                "ramptemp":   "n/a",
                "targettemp": "n/a",
                "segtime": "0:00:00",
            }

        # No data at all
        return {
            "run_profile": "none",
            "run_segment": "n/a",
            "status": "idle",
            "readtemp": "n/a",
            "ramptemp": "n/a",
            "targettemp": "n/a",
            "segtime": "0:00:00",
        }

    except Exception as e:
        # Never crash
        app.logger.exception("status_from_db failed")
        return {
            "run_profile": "none",
            "run_segment": "n/a",
            "status": "idle",
            "readtemp": "n/a",
            "ramptemp": "n/a",
            "targettemp": "n/a",
            "segtime": "0:00:00",
            "_error": str(e),
        }


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

app = Flask(
    __name__,
    template_folder="/home/pi/PILN/template",   # <-- add this
    # static_folder can stay default; lighttpd is serving /style itself
#	static_folder=os.path.join(APP_DIR, "style"),
#    static_url_path="/style",   
)

# -------- helpers --------

# Helper function to format timestamp without timezone conversion
def to_local_time(db_timestamp):
    if db_timestamp:
        # Check if the input is a datetime object
        if isinstance(db_timestamp, datetime):
            # Format the datetime object directly
            return db_timestamp.strftime('%Y-%m-%d %H:%M:%S')
        else:
            # If it's a string, return it as-is (assuming it matches the desired format)
            return db_timestamp
    return None

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

# Get segment start/end times for automatic updates
@app.get("/run/<int:run_id>/times")
def run_times(run_id):
    db = get_db()

    run = db.execute(
        "SELECT start_time, end_time, state FROM profiles WHERE run_id=?",
        (run_id,)
    ).fetchone()
    if run is None:
        return jsonify({"error": "not found"}), 404

    seg_rows = db.execute(
        "SELECT segment, start_time, end_time "
        "FROM segments WHERE run_id=? ORDER BY segment ASC",
        (run_id,)
    ).fetchall()

    segments = [
        {
            "segment": r["segment"],
            "start_time": to_local_time(r["start_time"]),
            "end_time": to_local_time(r["end_time"]),
        }
        for r in seg_rows
    ]

    return jsonify({
        "run_id": run_id,
        "state": run["state"],
        "start_time": to_local_time(run["start_time"]),
        "end_time": to_local_time(run["end_time"]),
        "segments": segments,
    })
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
        
# ---------- UI helpers ----------
from flask import render_template, request

def render_page(title: str, body_template: str, **ctx):
    """Stitch header + body + footer (keeps your current template structure)."""
    hdr = render_template("header.html", title=title)
    bdy = render_template(body_template, **ctx)
    ftr = render_template("footer.html")
    return hdr + bdy + ftr

# ---------- UI: HOME ----------
@app.get("/")
@app.get("/home")
def ui_home():
    db = get_db()
    rows = db.execute("""
        SELECT state, run_id, notes, start_time AS lastdate,
               CASE state
                 WHEN 'Running' THEN 0
                 WHEN 'Staged'  THEN 1
                 WHEN 'Stopped' THEN 2
                 WHEN 'Completed' THEN 3
               END AS ord_key
        FROM profiles
        ORDER BY ord_key ASC, run_id DESC
    """).fetchall()
    return render_page("Profile List", "home.html", profiles=rows)

# ---------- UI: VIEW PROFILE ----------
# ---------- UI: VIEW PROFILE ----------
@app.route("/view", methods=["GET", "POST"])
def ui_view():
    # request.values merges args (GET) and form (POST)
    run_id_raw = request.values.get("run_id", "0")
    state  = request.values.get("state", "")
    notes  = request.values.get("notes", "")

    try:
        run_id = int(run_id_raw)
    except (TypeError, ValueError):
        run_id = 0

    db = get_db()
    profile = db.execute("SELECT * FROM profiles WHERE run_id = ?", (run_id,)).fetchone()
    segments = db.execute("""
        SELECT segment, set_temp, rate, hold_min, int_sec, start_time, end_time
        FROM segments
        WHERE run_id = ?
        ORDER BY segment
    """, (run_id,)).fetchall()

    if state in ("Completed", "Stopped", "Error"):
        viewtmpl = "view_comp.html"
    elif state == "Running":
        viewtmpl = "view_run.html"
    else:
        viewtmpl = "view_staged.html"

    page = render_template(viewtmpl, segments=segments, profile=profile,
                           run_id=run_id, state=state, notes=notes)

    if state in ("Completed", "Running", "Stopped", "Error"):
        page += render_template("chart.html", run_id=run_id, notes=notes)

    return render_template("header.html", title="Profile Details") + page + render_template("footer.html")

# ---------- UI: NEW PROFILE FORM ----------
@app.get("/new")
def ui_new():
    maxsegs = 20
    segments = range(1, maxsegs + 1)
    return render_page("New Profile", "new.html", segments=segments)

# ---------- UI: EDIT/COPY ----------
@app.get("/editcopy")
def ui_editcopy():
    run_id = int(request.args.get("run_id", "0"))
    db = get_db()

    segments = db.execute("""
        SELECT segment, set_temp, rate, hold_min, int_sec
        FROM segments WHERE run_id = ? ORDER BY segment
    """, (run_id,)).fetchall()

    curcount = len(segments)
    maxsegs = 20
    addsegs = range(curcount + 1, maxsegs + 1)
    lastseg = curcount

    profile = db.execute(
        "SELECT notes, p_param, i_param, d_param FROM profiles WHERE run_id = ?",
        (run_id,)
    ).fetchone()

    state  = request.args.get("state", "")
    notes  = request.args.get("notes", "")

    return render_page("Edit/Copy Profile", "editcopy.html",
                       segments=segments, addsegs=addsegs, lastseg=lastseg,
                       run_id=run_id, profile=profile, state=state, notes=notes)

# ---------- UI: RUN PROFILE ----------
@app.post("/run")
def ui_run():
    run_id = int(request.form.get("run_id", "0"))
    notes  = request.form.get("notes", "")

    db = get_db()
    running = db.execute("SELECT run_id FROM profiles WHERE state='Running'").fetchone()
    if running:
        msg = f"Unable start profile - Profile {int(running['run_id'])} already running"
        body = render_template("reload.html", target_page="view", timeout=5000,
                               message=msg, params={"run_id": run_id, "state": "Staged", "notes": notes})
        return render_template("header.html", title="Run Profile") + body + render_template("footer.html")

    db.execute("UPDATE profiles SET state = ? WHERE run_id = ?", ("Running", run_id))
    db.commit()
    body = render_template("reload.html", target_page="view", timeout=800,
                           message="Updating profile to running state...",
                           params={"run_id": run_id, "state": "Running", "notes": notes})
    return render_template("header.html", title="Run Profile") + body + render_template("footer.html")

# ---------- UI: SAVE NEW / UPDATE ----------
@app.post("/savenew")
@app.post("/saveupd")
def ui_save():
    maxsegs = 20
    def_rate, def_holdmin, def_intsec = 9999, 0, 30

    db = get_db()
    page = request.path.rsplit("/", 1)[-1]  # savenew or saveupd

    p_param = float(request.form.get("Kp", 0.0))
    i_param = float(request.form.get("Ki", 0.0))
    d_param = float(request.form.get("Kd", 0.0))
    notes   = request.form.get("notes", "")

    if page == "savenew":
        cur = db.execute(
            "INSERT INTO profiles (state, notes, p_param, i_param, d_param) VALUES (?,?,?,?,?)",
            ("Staged", notes, p_param, i_param, d_param)
        )
        run_id = cur.lastrowid
    else:
        run_id = int(request.form.get("run_id", "0"))
        db.execute(
            "UPDATE profiles SET notes=?, p_param=?, i_param=?, d_param=? WHERE run_id=?",
            (notes, p_param, i_param, d_param, run_id)
        )
        db.execute("DELETE FROM segments WHERE run_id=?", (run_id,))

    # insert segments
    seg_rows = []
    for num in range(1, maxsegs + 1):
        seg = str(num)
        set_temp = request.form.get("set_temp" + seg, "")
        rate     = request.form.get("rate" + seg, "")
        hold_min = request.form.get("hold_min" + seg, "")
        int_sec  = request.form.get("int_sec" + seg, "")
        if set_temp != "":
            if rate == "":     rate = def_rate
            if hold_min == "": hold_min = def_holdmin
            if int_sec == "":  int_sec  = def_intsec
            seg_rows.append((run_id, num, int(set_temp), int(rate), int(hold_min), int(int_sec)))

    if seg_rows:
        db.executemany("""
            INSERT INTO segments (run_id, segment, set_temp, rate, hold_min, int_sec)
            VALUES (?,?,?,?,?,?)""", seg_rows)

    db.commit()

    body = render_template("reload.html", target_page="view", timeout=1000,
                           message="Saving profile...",
                           params={"state": "Staged", "run_id": run_id, "notes": notes})
    return render_template("header.html", title="Save Profile") + body + render_template("footer.html")

# ---------- UI: DELETE CONFIRM ----------
@app.get("/del_conf")
def ui_del_conf():
    run_id = int(request.args.get("run_id", "0"))
    state  = request.args.get("state", "")
    notes  = request.args.get("notes", "")

    db = get_db()
    segs = db.execute("""
        SELECT segment, set_temp, rate, hold_min, int_sec, start_time, end_time
        FROM segments WHERE run_id=? ORDER BY segment
    """, (run_id,)).fetchall()

    return render_page("Confirm Profile Delete", "del_conf.html",
                       segments=segs, run_id=run_id, notes=notes, state=state)

# ---------- UI: DELETE ----------
@app.post("/delete")
def ui_delete():
    run_id = int(request.form.get("run_id", "0"))
    db = get_db()
    db.execute("DELETE FROM firing  WHERE run_id=?", (run_id,))
    db.execute("DELETE FROM segments WHERE run_id=?", (run_id,))
    db.execute("DELETE FROM profiles WHERE run_id=?", (run_id,))
    db.commit()

    body = render_template("reload.html", target_page="home", timeout=1000,
                           message="Deleting profile...", params={"run_id": run_id})
    return render_template("header.html", title="Delete Profile") + body + render_template("footer.html")

# ---------- UI: STOP ----------
@app.post("/stop")
def ui_stop():
    run_id = int(request.form.get("run_id", "0"))
    db = get_db()
    db.execute("UPDATE profiles SET state=? WHERE run_id=?", ("Stopped", run_id))
    db.commit()

    body = render_template("reload.html", target_page="home", timeout=1000,
                           message="Updating profile...", params={"run_id": run_id})
    return render_template("header.html", title="Stop Profile Run") + body + render_template("footer.html")

# ---------- UI: SAVE NOTES ----------
@app.post("/notes_save")
def ui_notes_save():
    run_id = int(request.form.get("run_id", "0"))
    notes  = request.form.get("notes", "")
    state  = request.form.get("state", "")

    db = get_db()
    db.execute("UPDATE profiles SET notes=? WHERE run_id=?", (notes, run_id))
    db.commit()

    body = render_template("reload.html", target_page="view", timeout=0,
                           message="Saving notes...",
                           params={"state": state, "run_id": run_id, "notes": notes})
    return render_template("header.html", title="Save Notes") + body + render_template("footer.html")



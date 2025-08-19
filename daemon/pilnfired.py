#!/usr/bin/env python3
# pilnfired_refactored.py (online FOPDT adjust, no relay autotune)

from signal import signal, SIGABRT, SIGINT, SIGTERM
import os, time, math, logging, sqlite3, sys
from typing import Optional, Tuple, List
from datetime import datetime

import RPi.GPIO as GPIO
import board, busio, digitalio
import adafruit_max31856

# ----- SciPy (optional) -----
try:
    from scipy.optimize import curve_fit
    _SCIPY_OK = True
except Exception:
    _SCIPY_OK = False

# numpy (used by adapter)
import numpy as np

# -------------------
# Hardcoded paths
# -------------------
AppDir   = '/home/pi/PILN'
SQLDB    = '/home/pi/PILN/db/PiLN.sqlite3'

# -------------------
# Constants / Options
# -------------------
HEAT_PINS = (5, 6)
MAX_TEMP_C = 1330.0
MIN_VALID_TEMP_C = 0.1
DEBUG_SIM = True

# --- PID defaults ---
DEFAULT_KP = 20.0
DEFAULT_KI = 0.2
DEFAULT_KD = 0.2

# Online FOPDT adjuster (ON/OFF)
ONLINE_FOPDT_ADAPT     = True     # <- master switch (set False to disable all adjustment)
ERROR_C_FOR_ADAPT      = 5.0      # ramp error threshold (°C) to attempt an adjust
ADAPT_MIN_STEP         = 0.10     # >= 10% duty step between windows required
ADAPT_FIT_SECONDS      = 30*60    # fit last 30 minutes around the step window
ADAPT_LAMBDA_FACTOR    = 1.0      # SIMC lambda factor
ADAPT_MAX_GAIN_BLEND   = 0.25     # blend factor (new = old*(1-a) + suggested*a)
ADAPT_KP_BOUNDS        = (0.01, 200.0)
ADAPT_KI_BOUNDS        = (0.0,  50.0)   # per-window Ki bounds (note: Ki here is per call)
ADAPT_KD_BOUNDS        = (0.0, 200.0)   # we keep Kd unchanged by default

SAMPLE_QUIET_MS = 150
THERMOCOUPLE = "S"

# -------------------
# Logging
# -------------------
L = logging.getLogger("kiln")
L.setLevel(logging.DEBUG)

def configure_run_logger(run_id: Optional[int]):
    for h in L.handlers[:]:
        L.removeHandler(h)
    os.makedirs(os.path.join(AppDir, "log"), exist_ok=True)
    if run_id is not None:
        logfile = os.path.join(AppDir, 'log', f'RunID_{run_id}_Fired.log')
    else:
        logfile = os.path.join(AppDir, 'log', 'pilnfired_boot.log')
    fh = logging.FileHandler(logfile, 'a')
    fh.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    L.addHandler(fh)


#checks to see if pid constants are 0 and if so uses defaults
def _fallback_pid(kp, ki, kd):
    """Return (kp, ki, kd) with per-parameter fallback if value is None/NaN/0."""
    def _nz(v, dflt):
        try:
            if v is None: return dflt
            v = float(v)
            if math.isnan(v) or abs(v) < 1e-12:  # treat 0 (or near) as unset
                return dflt
            return v
        except Exception:
            return dflt
    used = []
    new_kp = _nz(kp, DEFAULT_KP);  used += (["Kp"] if (new_kp != kp) else [])
    new_ki = _nz(ki, DEFAULT_KI);  used += (["Ki"] if (new_ki != ki) else [])
    new_kd = _nz(kd, DEFAULT_KD);  used += (["Kd"] if (new_kd != kd) else [])
    if used:
        L.info(f"PID fallback for: {', '.join(used)}  -> Kp={new_kp}, Ki={new_ki}, Kd={new_kd}")
    return new_kp, new_ki, new_kd

# -------------------
# Hardware
# -------------------
class KilnHardware:
    def __init__(self, pins: Tuple[int, int] = HEAT_PINS):
        self.pins = pins
        self.last_cmd_on = False
        GPIO.setmode(GPIO.BCM)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)

    def heat_on(self):
        for p in self.pins:
            GPIO.output(p, GPIO.HIGH)
        self.last_cmd_on = True

    def heat_off(self):
        for p in self.pins:
            GPIO.output(p, GPIO.LOW)
        self.last_cmd_on = False

    def write_output_percent(self, pct: float):
        self.heat_on() if pct >= 50.0 else self.heat_off()

    def pause_for_sample(self, quiet_ms: int = SAMPLE_QUIET_MS):
        prev = self.last_cmd_on
        self.heat_off()
        time.sleep(quiet_ms / 1000.0)
        if prev:
            self.heat_on()

    def cleanup(self):
        try:
            self.heat_off()
        finally:
            GPIO.cleanup()

# -------------------
# Thermocouple
# -------------------
class ThermocoupleError(Exception):
    pass

class ThermocoupleReader:
    """Reads MAX31856 with progressive recovery and brief 'quiet' sampling to reduce EMI."""
    def __init__(self, hardware: KilnHardware,
                 soft_retries=3, reinit_attempts=2, hwreset_attempts=1, read_delay=0.2):
        self.hardware = hardware
        self.soft_retries = soft_retries
        self.reinit_attempts = reinit_attempts
        self.hwreset_attempts = hwreset_attempts
        self.read_delay = read_delay
        self._init_sensor()

    def _init_sensor(self):
        self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
        self.cs = digitalio.DigitalInOut(board.D0)   # CS on D0 per your wiring
        self.cs.direction = digitalio.Direction.OUTPUT
        th = getattr(adafruit_max31856.ThermocoupleType, THERMOCOUPLE.upper(),
                     adafruit_max31856.ThermocoupleType.S)
        self.tc = adafruit_max31856.MAX31856(self.spi, self.cs, thermocouple_type=th)

    def _full_reinit(self):
        try:
            self.cs.value = True
        except Exception:
            pass
        time.sleep(0.05)
        self._init_sensor()

    def _valid(self, t: float) -> bool:
        return (t is not None) and (not math.isnan(t)) and (MIN_VALID_TEMP_C <= t <= MAX_TEMP_C)

    def _read_once(self) -> float:
        # Quiet sample
        try:
            self.hardware.pause_for_sample(SAMPLE_QUIET_MS)
        except Exception:
            prev = getattr(self.hardware, "last_cmd_on", False)
            try:
                self.hardware.heat_off()
                time.sleep(SAMPLE_QUIET_MS / 1000.0)
            finally:
                if prev:
                    self.hardware.heat_on()

        t = float(self.tc.temperature)
        try:
            faults = self.tc.fault
            if isinstance(faults, int) and faults:
                L.warning("Thermocouple fault bitmask: 0x%02X", faults)
            elif isinstance(faults, dict) and any(faults.values()):
                L.warning("Thermocouple fault flags: %s", faults)
        except Exception:
            pass
        return t

    def read_temperature(self) -> Tuple[float, float]:
        for i in range(self.soft_retries):
            try:
                t = self._read_once()
                if self._valid(t):
                    return t, float(self.tc.reference_temperature)
                L.debug(f"Soft retry {i+1}/{self.soft_retries} invalid t={t}")
            except Exception as e:
                L.debug(f"Soft retry {i+1} exception {e}")
            time.sleep(self.read_delay)

        for i in range(self.reinit_attempts):
            try:
                L.warning("Reinitializing MAX31856 object…")
                self._init_sensor()
                t = self._read_once()
                if self._valid(t):
                    return t, float(self.tc.reference_temperature)
            except Exception as e:
                L.debug(f"Reinit {i+1} exception {e}")
            time.sleep(self.read_delay)

        for i in range(self.hwreset_attempts):
            try:
                L.warning("Full hardware reinit for thermocouple/SPI…")
                self._full_reinit()
                t = self._read_once()
                if self._valid(t):
                    return t, float(self.tc.reference_temperature)
            except Exception as e:
                L.debug(f"HW reset {i+1} exception {e}")
            time.sleep(self.read_delay)

        L.error("Thermocouple permanently failed; failing safe. Relays OFF.")
        try:
            self.hardware.heat_off()
        except Exception:
            pass
        raise ThermocoupleError("MAX31856 read failed after recovery attempts")

# -------------------
# PID Controller
# -------------------
class PIDController:
    """Time-proportioning PID; Ki is per-call (per window); no explicit dt."""
    def __init__(self, kp: float, ki: float, kd: float, i_min: float, i_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_min = i_min
        self.i_max = i_max
        self._i_term = 0.0
        self._last_proc = 0.0

    def compute(self, setpoint: float, pv: float) -> float:
        err = setpoint - pv
        self._i_term += self.ki * err
        if self._i_term > self.i_max: self._i_term = self.i_max
        if self._i_term < self.i_min: self._i_term = self.i_min
        d_input = pv - self._last_proc
        out = self.kp * err + self._i_term - self.kd * d_input
        if out > self.i_max: out = self.i_max
        if out < self.i_min: out = self.i_min
        if err > 200:
            out = 200.0
        self._last_proc = pv
        L.debug(f"PID compute -> err={err:.2f} I={self._i_term:.2f} dIn={d_input:.2f} out={out:.2f}")
        return out

    def reset(self):
        self._i_term = 0.0
        self._last_proc = 0.0

# -------------------
# Online FOPDT Adapter (once per window)
# -------------------
def _fopdt_step_rel(t, K, tau, theta, dU):
    t = np.asarray(t, dtype=float)
    y = np.zeros_like(t, dtype=float)
    tau = max(float(tau), 1e-6)
    idx = t > theta
    y[idx] = K * dU * (1.0 - np.exp(-(t[idx] - theta) / tau))
    return y

def _fit_fopdt_unit_step(t_rel, y_rel, dU):
    # local import to avoid hard dependency if SciPy missing at import time
    from scipy.optimize import curve_fit as _cf
    import numpy as _np
    m = _np.isfinite(t_rel) & _np.isfinite(y_rel)
    t_rel = _np.asarray(t_rel)[m]; y_rel = _np.asarray(y_rel)[m]
    ord_ = _np.argsort(t_rel); t_rel = t_rel[ord_]; y_rel = y_rel[ord_]
    keep = _np.insert(_np.diff(t_rel) > 0, 0, True)
    t_rel = t_rel[keep]; y_rel = y_rel[keep]
    if t_rel.size < 8:
        raise RuntimeError("Too few points for FOPDT fit")
    if abs(dU) < 1e-6:
        raise RuntimeError("No effective input step (dU≈0)")

    y_end = float(y_rel[-1]); K0 = y_end / dU
    tau0 = max(60.0, (t_rel[-1] - t_rel[0]) / 3.0)
    theta0 = max(0.0, _np.median(_np.diff(t_rel)))
    K_lo, K_hi = -1e4, 1e4
    tau_lo, tau_hi = 1.0, 1e6
    th_lo, th_hi = 0.0, 7200.0

    def clamp(v, lo, hi):
        if not _np.isfinite(v): return 0.5*(lo+hi)
        return min(max(v, lo + 1e-9), hi - 1e-9)

    p0 = [clamp(K0, K_lo, K_hi), clamp(tau0, tau_lo, tau_hi), clamp(theta0, th_lo, th_hi)]
    bounds = ([K_lo, tau_lo, th_lo], [K_hi, tau_hi, th_hi])

    def model_wrapped(t, K, tau, theta):
        return _fopdt_step_rel(t, K, tau, theta, dU)

    popt, _ = _cf(model_wrapped, t_rel, y_rel, p0=p0, bounds=bounds, maxfev=20000)
    K, tau, theta = map(float, popt)
    return K, tau, theta

def _simc_pi_from_fopdt(K, tau, theta, lambda_factor=1.0):
    lam = max(theta, lambda_factor * tau)
    if abs(K) < 1e-9:
        raise RuntimeError("Process gain ~ 0; cannot compute PI")
    Kp = tau / (K * (lam + theta))
    Ti = min(tau, 4.0 * (lam + theta))
    Ki_sec = Kp / max(Ti, 1e-6)     # per second
    return Kp, Ki_sec, lam

class OnlineFOPDTAdapter:
    """
    Stores one sample per window and (optionally) adjusts Kp/Ki for next window.
    - Triggers only if |ramp - temp| >= ERROR_C_FOR_ADAPT AND |Δduty| >= ADAPT_MIN_STEP.
    - Uses last ADAPT_FIT_SECONDS around the detected step.
    - Blended, clamped Kp/Ki; leaves Kd unchanged.
    """
    def __init__(self, window_sec: int):
        self.window = int(max(1, window_sec))
        self.t: List[float] = []
        self.y: List[float] = []
        self.ramp: List[float] = []
        self.u: List[float] = []   # 0..1 duty

        if not _SCIPY_OK and ONLINE_FOPDT_ADAPT:
            L.warning("SciPy not available; ONLINE_FOPDT_ADAPT will be ignored.")

    def add_sample(self, ts: float, pv: float, ramp_temp: float, duty_pct: float):
        self.t.append(float(ts))
        self.y.append(float(pv))
        self.ramp.append(float(ramp_temp))
        self.u.append(max(0.0, min(1.0, float(duty_pct)/100.0)))

        # house-keep: keep only last 6 hours to bound memory
        cutoff = ts - 6*3600
        while self.t and self.t[0] < cutoff:
            self.t.pop(0); self.y.pop(0); self.ramp.pop(0); self.u.pop(0)

    def maybe_update_pid(self, pid: PIDController) -> bool:
        if not ONLINE_FOPDT_ADAPT or not _SCIPY_OK:
            return False
        n = len(self.t)
        if n < 3:   # need at least two prior windows
            return False

        # recent ramp error and duty step across the last two windows
        err = self.ramp[-1] - self.y[-1]
        dU  = self.u[-1] - self.u[-2]

        if abs(err) < ERROR_C_FOR_ADAPT or abs(dU) < ADAPT_MIN_STEP:
            return False

        # choose window starting at the step index (n-1), include up to ADAPT_FIT_SECONDS after
        t0 = self.t[-2]   # time of previous window end (where step occurred)
        i0 = n-2
        # collect indices within fit horizon
        i_end = i0
        while (i_end + 1) < n and (self.t[i_end] - t0) < ADAPT_FIT_SECONDS:
            i_end += 1
        if (i_end - i0) < 3:
            return False

        t_rel = np.array(self.t[i0:i_end+1], dtype=float) - t0
        y_win = np.array(self.y[i0:i_end+1], dtype=float)

        # baseline PV before step (median of few prior points if available)
        pre0 = max(0, i0-3)
        y0 = float(np.median(self.y[pre0:i0])) if i0 > pre0 else float(self.y[i0])
        y_rel = y_win - y0

        try:
            K, tau, theta = _fit_fopdt_unit_step(t_rel, y_rel, dU)
        except Exception as e:
            L.debug(f"[ADAPT] FOPDT fit failed: {e}")
            return False

        try:
            Kp_suggest, Ki_sec, lam = _simc_pi_from_fopdt(K, tau, theta, ADAPT_LAMBDA_FACTOR)
        except Exception as e:
            L.debug(f"[ADAPT] SIMC gains failed: {e}")
            return False

        # Convert Ki to per-window "one call per window" units
        Ki_call = Ki_sec * self.window

        # Blend & clamp
        def clamp(v, lo, hi): return max(lo, min(hi, v))
        a = ADAPT_MAX_GAIN_BLEND
        new_kp = clamp(pid.kp*(1-a) + Kp_suggest*a, *ADAPT_KP_BOUNDS)
        new_ki = clamp(pid.ki*(1-a) + Ki_call*a,   *ADAPT_KI_BOUNDS)
        new_kd = pid.kd  # leave unchanged

        if (abs(new_kp - pid.kp) < 1e-12) and (abs(new_ki - pid.ki) < 1e-12):
            return False

        L.info(f"[ADAPT] err={err:+.1f}°C dU={dU:+.2f}  FOPDT: K={K:.3g} τ={tau:.1f}s θ={theta:.1f}s λ={lam:.1f}s  "
               f"Kp {pid.kp:.4g}->{new_kp:.4g}  Ki {pid.ki:.4g}->{new_ki:.4g} (per window)")
        pid.kp, pid.ki, pid.kd = new_kp, new_ki, new_kd
        return True

# -------------------
# Kiln Controller
# -------------------
class KilnController:
    def __init__(self):
        self.hardware = KilnHardware()
        self.tc_reader = ThermocoupleReader(self.hardware)
        self.sql = sqlite3.connect(SQLDB)
        self.sql.row_factory = sqlite3.Row
        self.cur = self.sql.cursor()

        # Index to speed up idle upsert (safe if it already exists)
        try:
            self.cur.execute("CREATE INDEX IF NOT EXISTS idx_firing_run_seg ON firing(run_id, segment)")
            self.sql.commit()
        except Exception:
            self.sql.rollback()

        self._sim_booted = False  # one-shot flag for sim init per run
        self.pid = PIDController(0.0, 0.0, 0.0, i_min=0.0, i_max=100.0)
        self.sim_temp = 25.0

        for sig in (SIGABRT, SIGINT, SIGTERM):
            signal(sig, self._clean_exit)

        configure_run_logger(None)
        L.info("=== START PiLN Firing Daemon (FOPDT online adjust) ===")

    def _clean_exit(self, *args):
        print("\nProgram ending! Cleaning up...\n")
        try:
            self.hardware.heat_off()
            self.hardware.cleanup()
        finally:
            print("All clean - Stopping.\n")
            os._exit(0)

    def read_temp(self) -> Tuple[float, float]:
        """Unified read: returns (process_temp, internal_temp). In sim, returns simulated value."""
        if DEBUG_SIM:
            return self.sim_temp, 25.0
        else:
            return self.tc_reader.read_temperature()

    # --- DB helpers ---
    def set_profile_start(self, run_id: int):
        st = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cur.execute("UPDATE profiles SET start_time=? WHERE run_id=?;", (st, run_id))
        self.sql.commit()

    def set_profile_end(self, run_id: int, state: str):
        et = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cur.execute("UPDATE profiles SET end_time=?, state=? WHERE run_id=?;", (et, state, run_id))
        self.sql.commit()

    def set_segment_start(self, run_id: int, seg: int):
        st = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cur.execute("UPDATE segments SET start_time=? WHERE run_id=? AND segment=?;", (st, run_id, seg))
        self.sql.commit()

    def set_segment_end(self, run_id: int, seg: int):
        et = time.strftime('%Y-%m-%d %H:%M:%S')
        self.cur.execute("UPDATE segments SET end_time=? WHERE run_id=? AND segment=?;", (et, run_id, seg))
        self.sql.commit()

    def insert_firing_row(self, run_id, seg, set_temp, temp, int_temp, pid_out):
        try:
            self.cur.execute(
                "INSERT INTO firing (run_id, segment, dt, set_temp, temp, int_temp, pid_output) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (run_id, seg, time.strftime('%Y-%m-%d %H:%M:%S'),
                 float(set_temp), float(temp), float(int_temp), float(pid_out))
            )
            self.sql.commit()
        except Exception:
            self.sql.rollback()

    def upsert_idle_temp(self, temp, int_temp):
        """Keep a single idle row (run_id=0, segment=0) updated while idle."""
        now = time.strftime('%Y-%m-%d %H:%M:%S')
        try:
            self.cur.execute(
                "UPDATE firing "
                "SET dt=?, set_temp=0, temp=?, int_temp=?, pid_output=0.0 "
                "WHERE run_id=0 AND segment=0",
                (now, float(temp), float(int_temp))
            )
            if self.cur.rowcount == 0:
                try:
                    self.cur.execute(
                        "INSERT INTO firing (run_id, segment, dt, set_temp, temp, int_temp, pid_output) "
                        "VALUES (0, 0, ?, 0, ?, ?, 0.0)",
                        (now, float(temp), float(int_temp))
                        )
                except Exception:
                    print("firing row 0 could not be created")
            self.sql.commit()
        except Exception:
            print("The Idle temps could not be updated or sql row could not be created")
            self.sql.rollback()

    # ---- main segment firing loop ----
    def fire_segment(self, run_id: int, seg: int, target_tmp: float, rate: float,
                     hold_min: int, window: int, kp: float, ki: float, kd: float):

        L.info(f"Entering segment RunID={run_id} Seg={seg} Target={target_tmp} Rate={rate} HoldMin={hold_min} Window={window}")

        # Apply initial DB gains; note Ki in DB is per-call (as before)
        self.pid = PIDController(kp, ki, kd, i_min=0.0, i_max=100.0)

        # Initialize simulated ambient once per run, at segment 1
        if DEBUG_SIM and seg == 1 and not self._sim_booted:
            self.sim_temp = 20.0
            self._sim_booted = True
            L.info("[SIM] Initializing sim_temp to 20.0°C for segment 1")

        adapter = OnlineFOPDTAdapter(window)

        run_state = "Ramp"

        # initial read (always use unified reader so sim updates propagate)
        try:
            read_tmp, int_tmp = self.read_temp()
        except ThermocoupleError:
            run_state = "Error"
            try: self.hardware.heat_off()
            except Exception: pass
            return run_state

        last_tmp = 0.0
        last_err = 0.0
        start_tmp = 0.0
        tmp_dif = 0.0
        steps = 0.0
        step_tmp = 0.0
        start_sec = 0
        end_sec = 0
        next_sec = time.time()
        cnt = 0

        ramp_tmp = 0.0
        ramp_trg = 0
        read_trg = 0

        while run_state not in ("Stopped", "Complete", "Error"):
            now = time.time()
            if now >= next_sec:
                cnt += 1
                next_sec = now + window
                last_tmp = read_tmp

                # fresh reading (unconditional)
                try:
                    read_tmp, int_tmp = self.read_temp()
                except ThermocoupleError:
                    run_state = "Error"
                    try: self.hardware.heat_off()
                    except Exception: pass
                    break

                # guard bad read
                if math.isnan(read_tmp) or read_tmp > MAX_TEMP_C:
                    read_tmp = last_tmp + last_err
                    print('  "kilntemp": "' + str(int(read_tmp)) + '",\n')

                # first pass (init ramp math)
                if start_tmp == 0:
                    start_tmp = read_tmp
                    start_sec = int(time.time())
                    next_sec = start_sec + window
                    tmp_dif = target_tmp - start_tmp
                    ramp_min = abs(tmp_dif) * 60.0 / rate if rate > 0 else 0.0
                    steps = (ramp_min * 60.0 / window) if window > 0 else 1.0
                    step_tmp = (tmp_dif / steps) if steps > 0 else 0.0
                    end_sec = start_sec + int(ramp_min*60.0) + int(hold_min*60)
                    ramp_tmp = start_tmp + step_tmp
                    if ((tmp_dif > 0 and ramp_tmp > target_tmp) or (tmp_dif < 0 and ramp_tmp < target_tmp)):
                        ramp_tmp = target_tmp
                    last_err = 0.0
                    L.info(f"First pass: Target={target_tmp:.2f} Start={start_tmp:.2f} RampTmp={ramp_tmp:.2f} TmpDif={tmp_dif:.2f} "
                           f"Steps={int(steps)} StepTmp={step_tmp:.2f} Window={window} StartSec={start_sec} EndSec={end_sec}")

                # ramp advance unless already at target
                if ramp_trg == 0:
                    ramp_tmp += step_tmp

                # Rising logic
                if tmp_dif > 0:
                    if ramp_trg == 0 and ramp_tmp >= target_tmp:
                        ramp_tmp = target_tmp; ramp_trg = 1
                        run_state = "Ramp complete" if read_trg == 0 else "Ramp/Hold"
                    if ((target_tmp - read_tmp) <= 0.5 or read_tmp >= target_tmp) and read_trg == 0:
                        read_trg = 1
                        end_sec = int(time.time()) + hold_min*60
                        L.info(f"Set temp reached - End seconds set to {end_sec}")
                        run_state = "Target Reached" if ramp_trg == 0 else "Target/Hold"

                # Falling logic
                elif tmp_dif < 0:
                    if ramp_tmp <= target_tmp and ramp_trg == 0:
                        ramp_tmp = target_tmp; ramp_trg = 1
                        run_state = "Ramp complete" if read_trg == 0 else "Target/Ramp"
                    if ((read_tmp - target_tmp) <= 0.5 or read_tmp <= target_tmp) and read_trg == 0:
                        read_trg = 1
                        end_sec = int(time.time()) + hold_min*60
                        L.info(f"Set temp reached - End seconds set to {end_sec}")
                        run_state = "Target Reached" if ramp_trg == 0 else "Ramp/Target"

                # PID compute, convert to duty
                out = self.pid.compute(ramp_tmp, read_tmp)
                cycle_on_sec = min(window, window * (out * 0.01))

                remain_sec = end_sec - int(time.time())
                rem_min, rem_sec = divmod(max(0, remain_sec), 60)
                rem_hr, rem_min = divmod(rem_min, 60)
                rem_time = f"{rem_hr}:{rem_min:02d}:{rem_sec:02d}"

                L.debug(f"RunID {run_id}, Segment {seg} (loop {cnt}) - RunState:{run_state}, "
                        f"ReadTmp:{read_tmp:.2f}, RampTmp:{ramp_tmp:.2f}, TargetTmp:{target_tmp:.2f}, "
                        f"Output:{out:.2f}, CycleOnSec:{cycle_on_sec:.2f}, RemainTime:{rem_time}")

                print(f"""RunID {run_id}, Segment {seg} (loop {cnt}) - RunState:{run_state},
                       ReadTmp:{read_tmp:.2f}, RampTmp:{ramp_tmp:.2f}, TargetTmp:{target_tmp:.2f},
                       Output:{out:.2f}, CycleOnSec:{cycle_on_sec:.2f}, RemainTime:{rem_time}
                """)

                # drive relays (time-proportioning)
                if out > 0:
                    if out == 200.0:
                        self.cur.execute("UPDATE profiles SET state=? WHERE run_id=?;", ('Error', run_id))
                        self.sql.commit()
                        L.error(f"State = Error RunID: {run_id}")
                        run_state = "Error"
                        try: self.hardware.heat_off()
                        except Exception: pass
                    else:
                        if DEBUG_SIM:
                            self.hardware.heat_off()
                            # simplistic sim: heat raises temp proportional to on-time
                            self.sim_temp += (cycle_on_sec * 5.0)
                        else:
                            self.hardware.heat_on()
                        time.sleep(cycle_on_sec)

                if out < 100.0:
                    self.hardware.heat_off()
                    if DEBUG_SIM:
                        # simplistic sim: cool a bit when off
                        self.sim_temp -= 2.0

                # status + logging row
                try:
                    self.insert_firing_row(run_id, seg, ramp_tmp, read_tmp, int_tmp, out)
                except Exception:
                    self.sql.rollback()
                    L.error("DB insert failed (firing)")

                # --- Online FOPDT adjust (once per window) ---
                try:
                    adapter.add_sample(time.time(), read_tmp, ramp_tmp, out)
                    adapter.maybe_update_pid(self.pid)
                except Exception as e:
                    L.debug(f"[ADAPT] exception: {e}")

                # profile still running?
                self.cur.execute("SELECT * FROM profiles WHERE state='Running';")
                rows = self.cur.fetchall()
                if rows:
                    running_id = rows[0]['run_id']
                    if time.time() > end_sec and read_trg == 1:
                        run_state = "Complete"
                if (not rows or running_id != run_id) and run_state != "Error":
                    L.warning("Profile no longer in running state - exiting firing")
                    run_state = "Stopped"

                L.info(f"RunState end: {run_state}")

        return run_state

    # ---- poll loop ----
    def run(self):
        L.info("Polling for 'Running' firing profiles...")
        while True:
            try:
                read_tmp, int_tmp = self.read_temp()
            except ThermocoupleError:
                read_tmp, int_tmp = float('nan'), float('nan')

            self.cur.execute("SELECT * FROM profiles WHERE state=?;", ('Running',))
            data = self.cur.fetchall()

            if data:
                run_id = data[0]['run_id']
                kp = float(data[0]['p_param']) if data[0]['p_param'] is not None else 0.0
                ki = float(data[0]['i_param']) if data[0]['i_param'] is not None else 0.0
                kd = float(data[0]['d_param']) if data[0]['d_param'] is not None else 0.0

                kp, ki, kd = _fallback_pid(kp, ki, kd)

                configure_run_logger(run_id)
                L.info("=== Run start ===")

                if DEBUG_SIM:
                    self._sim_booted = False   # reset per run

                try:
                    self.set_profile_start(run_id)
                except Exception:
                    self.sql.rollback()

                # get segments
                self.cur.execute("SELECT * FROM segments WHERE run_id=?;", (run_id,))
                profsegs = self.cur.fetchall()
                L.info(f"TotalSeg: {len(profsegs)}")

                run_state_final = None

                for row in profsegs:
                    seg       = row['segment']
                    target    = row['set_temp']
                    rate      = row['rate']
                    hold_min  = row['hold_min']
                    window    = row['int_sec']

                    if row['start_time'] is not None and row['end_time'] is not None:
                        L.info(f"segment {seg} already finished")
                        continue

                    try: self.set_segment_start(run_id, seg)
                    except Exception: self.sql.rollback()

                    state = self.fire_segment(run_id, seg, target, rate, hold_min, window, kp, ki, kd)
                    run_state_final = state

                    try:
                        self.hardware.heat_off()  # ensure off between segments
                    except Exception:
                        pass

                    try: self.set_segment_end(run_id, seg)
                    except Exception: self.sql.rollback()

                    if state in ("Error", "Stopped"):
                        break

                if run_state_final not in ("Error", "Stopped"):
                    try:
                        self.set_profile_end(run_id, 'Completed')
                        L.info("state updated to Completed")
                    except Exception:
                        self.sql.rollback()
                        L.error("DB Update failed (set Completed)")

                configure_run_logger(None)
                L.info("Polling for 'Running' firing profiles...")
            else:
                # IDLE: maintain one up-to-date row
                if not math.isnan(read_tmp):
                    self.upsert_idle_temp(read_tmp, int_tmp)
                time.sleep(5)
                continue

            time.sleep(2)

# -------------------
# Entry point
# -------------------
def main():
    ctl = KilnController()
    try:
        ctl.run()
    finally:
        ctl.hardware.cleanup()

if __name__ == "__main__":
    main()

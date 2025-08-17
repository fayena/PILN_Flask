#!/usr/bin/env python3
# pilnfired_refactored.py
#
# Drop-in refactor of the original pilnfired.py:
# - Same hardcoded paths, GPIO pins, DB layout, and status file.
# - Class-based design (PID, Hardware, Thermocouple, KilnController).
# - Progressive thermocouple error recovery: soft-retry -> reinit -> full reinit -> fail-safe (no reboot).
# - Optional relay/Åström–Hägglund PID autotune (set AUTOTUNE_ON_START=True to enable).
#
# Notes:
# - Keeps time-proportioning window logic from the original.
# - Writes per-RUN_ID logs in AppDir/log like before.
# - Uses adafruit_max31856 via Blinka (same as original).
# - Designed to "work as is" with your existing SQLite schema.

from signal import signal, SIGABRT, SIGINT, SIGTERM
import os
import time
import math
import logging
import sqlite3
import sys
from typing import Optional, Tuple, List, Dict

import RPi.GPIO as GPIO
import board
import busio
import digitalio
import adafruit_max31856

# -------------------
# Hardcoded paths (kept the same)
# -------------------
AppDir = '/home/pi/PILN'
SQLDB = '/home/pi/PILN/db/PiLN.sqlite3'

# -------------------
# Constants / Options
# -------------------
HEAT_PINS = (5, 6)          # Same as original
MAX_TEMP_C = 1330.0           # sanity bound (like original guard)
MIN_VALID_TEMP_C = 0.1
DEBUG_SIM = True            # set True to simulate heating (like original Debug mode)
AUTOTUNE_ON_START = True     # set True to run relay autotune at the start of each run
AUTOTUNE_NOISE_BAND = 2.0     # deg C around setpoint for relay toggling
AUTOTUNE_MAX_SECONDS = 90 * 60
AUTOTUNE_MIN_HALF_CYCLES = 6  # ~3 cycles recommended
THERMOCOUPLE = "S"

# -------------------
# Logging
# -------------------
L = logging.getLogger("kiln")
L.setLevel(logging.DEBUG)  # handlers configured per-run

def configure_run_logger(run_id: Optional[int]):
    """Per-run log file like original: AppDir/log/RunID_<id>_Fired.log"""
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


# -------------------
# Hardware Abstraction
# -------------------
class KilnHardware:
    """Owns relay GPIO and provides helpers to turn heat on/off."""
    def __init__(self, pins: Tuple[int, int] = HEAT_PINS):
        self.pins = pins
        GPIO.setmode(GPIO.BCM)
        for p in self.pins:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, GPIO.LOW)  # fail safe off

    def heat_on(self):
        for p in self.pins:
            GPIO.output(p, GPIO.HIGH)

    def heat_off(self):
        for p in self.pins:
            GPIO.output(p, GPIO.LOW)

    def write_output_percent(self, pct: float):
        """For autotune relaying: >=50% => ON; else OFF."""
        if pct >= 50.0:
            self.heat_on()
        else:
            self.heat_off()

    def cleanup(self):
        try:
            self.heat_off()
        finally:
            GPIO.cleanup()


# -------------------
# Thermocouple Reader with Progressive Recovery
# -------------------
class ThermocoupleError(Exception):
    pass

class ThermocoupleReader:
    """Reads MAX31856 with progressive recovery and 'relays off before read' to reduce EMI."""
    def __init__(self, hardware: KilnHardware,
                 soft_retries=3, reinit_attempts=2, hwreset_attempts=1, read_delay=0.2):
        self.hardware = hardware
        self.soft_retries = soft_retries
        self.reinit_attempts = reinit_attempts
        self.hwreset_attempts = hwreset_attempts
        self.read_delay = read_delay
        self._init_sensor()

    def _init_sensor(self):
        self.spi = busio.SPI(board.SCK, board.MOSI, board.MISO)  # same wiring as before
        self.cs = digitalio.DigitalInOut(board.D0)               # your CS pin
        self.cs.direction = digitalio.Direction.OUTPUT
        th = getattr(adafruit_max31856.ThermocoupleType, str(THERMOCOUPLE).upper(), adafruit_max31856.ThermocoupleType.S)
        self.tc = adafruit_max31856.MAX31856(
            self.spi,
            self.cs,
            thermocouple_type=th
        )

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
        # turn relays off before reading to reduce coupling
        try:
            self.hardware.heat_off()
        except Exception:
            pass
        t = float(self.tc.temperature)
        try:
            faults = self.tc.fault
            if isinstance(faults, int):
                if faults:
                    L.warning("Thermocouple fault bitmask: 0x%02X", faults)
            elif isinstance(faults, dict) and any(faults.values()):
                L.warning("Thermocouple fault flags: %s", faults)
        except Exception:
            pass

        return t

    def read_temperature(self) -> Tuple[float, float]:
        """Returns (temp, internal_reference_temp). Raises on failure fail-safe (no reboot).."""
        # --- soft retries ---
        for i in range(self.soft_retries):
            try:
                t = self._read_once()
                if self._valid(t):
                    return t, float(self.tc.reference_temperature)
                L.debug(f"Soft retry {i+1}/{self.soft_retries} invalid t={t}")
            except Exception as e:
                L.debug(f"Soft retry {i+1} exception {e}")
            time.sleep(self.read_delay)

        # --- reinit object ---
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

        # --- full hardware reinit ---
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

        # --- last resort: fail-safe (no reboot). (match original behavior) ---
        L.error("Thermocouple permanently failed; failing safe. Relays OFF.")
        try:
            self.hardware.heat_off()
        except Exception:
            pass
        raise ThermocoupleError("MAX31856 read failed after recovery attempts")

# -------------------
# PID Controller (time-proportioning output)
# -------------------
class PIDController:
    """Drop-in replacement for Update(), with clamping and simple anti-windup."""
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
        # clamp I
        if self._i_term > self.i_max: self._i_term = self.i_max
        if self._i_term < self.i_min: self._i_term = self.i_min
        d_input = pv - self._last_proc
        out = self.kp * err + self._i_term - self.kd * d_input
        # output clamp to [i_min, i_max] percent-like range
        if out > self.i_max: out = self.i_max
        if out < self.i_min: out = self.i_min
        # safety: like original "Err > 200 => Output 200" (force 'error' condition)
        if err > 200:
            out = 200.0
        self._last_proc = pv

        L.debug(f"PID compute -> err={err:.2f} I={self._i_term:.2f} dIn={d_input:.2f} out={out:.2f}")
        return out

    def reset(self):
        self._i_term = 0.0
        self._last_proc = 0.0


# -------------------
# Relay Autotune (Åström–Hägglund)
# -------------------
class RelayAutotune:
    """
    Minimal relay autotune used only when AUTOTUNE_ON_START=True.
    Forces on/off around setpoint; measures oscillation to estimate Ku, Tu.
    Produces Tyreus–Luyben gains (gentler for thermal).
    """
    def __init__(self, setpoint: float, read_pv, write_out, noiseband=2.0,
                 max_seconds=AUTOTUNE_MAX_SECONDS, min_half_cycles=AUTOTUNE_MIN_HALF_CYCLES):
        self.SP = float(setpoint)
        self.read_pv = read_pv
        self.write_out = write_out
        self.band = float(noiseband)
        self.max_seconds = int(max_seconds)
        self.min_half_cycles = int(min_half_cycles)
        self._start = time.time()
        self._state_high = True
        self._extrema: List[Dict] = []  # {"ts":..., "pv":..., "type":"max"/"min"}
        self._extreme_pv = None

        # start heating
        self.write_out(100.0)
        L.info(f"[AUTOTUNE] start SP={self.SP:.1f}, band±{self.band:.1f}")

    def _switch_if_needed(self, pv: float) -> bool:
        if self._state_high and pv >= (self.SP + self.band):
            self._state_high = False
            self.write_out(0.0)
            self._record_extreme("max")
            L.info(f"[AUTOTUNE] switch->LOW pv={pv:.2f}")
            return True
        if (not self._state_high) and pv <= (self.SP - self.band):
            self._state_high = True
            self.write_out(100.0)
            self._record_extreme("min")
            L.info(f"[AUTOTUNE] switch->HIGH pv={pv:.2f}")
            return True
        return False

    def _record_extreme(self, typ: str):
        if self._extreme_pv is not None:
            self._extrema.append({"ts": time.time(), "pv": self._extreme_pv, "type": typ})
            self._extreme_pv = None

    def _track_extreme(self, pv: float):
        if self._extreme_pv is None:
            self._extreme_pv = pv
            return
        if self._state_high:
            # rising from a min
            if pv < self._extreme_pv: self._extreme_pv = pv
        else:
            # falling from a max
            if pv > self._extreme_pv: self._extreme_pv = pv

    def _enough(self) -> bool:
        # need >= 6 half-cycles for decent estimate
        return len(self._extrema) >= self.min_half_cycles

    def _analyze(self) -> Tuple[float, float, float]:
        maxima = [e for e in self._extrema if e["type"] == "max"]
        minima = [e for e in self._extrema if e["type"] == "min"]
        if len(maxima) < 2 or len(minima) < 2:
            raise RuntimeError("Insufficient extrema for analysis")

        # amplitude a
        pairs = min(len(maxima), len(minima))
        amps = [abs(maxima[-i]["pv"] - minima[-i]["pv"]) / 2.0 for i in range(1, pairs+1)]
        a = sum(amps) / len(amps)

        # period Tu (use maxima)
        periods = []
        for i in range(1, len(maxima)):
            dt = maxima[i]["ts"] - maxima[i-1]["ts"]
            if dt > 0: periods.append(dt)
        if not periods:
            raise RuntimeError("No valid period measured")
        Tu = sum(periods) / len(periods)

        # ultimate gain Ku from relay amplitude (high=100, low=0 => d=50% => 0.5 in [0..1])
        d = 0.5
        Ku = (4.0 * d) / (math.pi * a)

        L.info(f"[AUTOTUNE] a={a:.2f} Tu={Tu:.2f}s Ku={Ku:.4f}")
        return Ku, Tu, a

    @staticmethod
    def tyreus_luyben(Ku: float, Tu: float) -> Tuple[float, float, float]:
        Kp = 0.454 * Ku
        Ti = 2.2 * Tu
        Td = 0.159 * Tu
        Ki = Kp / Ti
        Kd = Kp * Td
        return Kp, Ki, Kd

    def run(self) -> Tuple[float, float, float]:
        try:
            while True:
                if (time.time() - self._start) > self.max_seconds:
                    raise RuntimeError("Autotune timed out")
                pv = float(self.read_pv())
                self._track_extreme(pv)
                switched = self._switch_if_needed(pv)
                if switched and self._enough():
                    break
                time.sleep(1.0)
            Ku, Tu, _ = self._analyze()
            kp, ki, kd = self.tyreus_luyben(Ku, Tu)
            return kp, ki, kd
        finally:
            # always turn off heat at end
            self.write_out(0.0)
            L.info("[AUTOTUNE] end")


# -------------------
# Kiln Controller (Profiles/Segments, Status, DB I/O)
# -------------------
class KilnController:
    def __init__(self):
        self.hardware = KilnHardware()
        self.tc_reader = ThermocoupleReader(self.hardware)
        self.sql = sqlite3.connect(SQLDB)
        self.sql.row_factory = sqlite3.Row
        self.cur = self.sql.cursor()

        # active PID (values will be set per run)
        self.pid = PIDController(0.0, 0.0, 0.0, i_min=0.0, i_max=100.0)

        # for Debug sim
        self.sim_temp = 20.0

        # cleanup on signals
        for sig in (SIGABRT, SIGINT, SIGTERM):
            signal(sig, self._clean_exit)

        # initial log (boot)
        configure_run_logger(None)
        L.info("=== START PiLN Firing Daemon (refactor)===")

    def _clean_exit(self, *args):
        print("\nProgram ending! Cleaning up...\n")
        try:
            self.hardware.heat_off()
            self.hardware.cleanup()
        finally:
            print("All clean - Stopping.\n")
            os._exit(0)

    # ---- temperature read with debug sim ----
    def read_temp(self) -> Tuple[float, float]:
        if DEBUG_SIM:
            print(self.sim_temp)
            return self.sim_temp, 25.0
        else:
            return self.tc_reader.read_temperature()

     # ---- status writer (disabled; UI reads DB via /api/status) ----
    def write_status(self, readtemp, run_id, seg, ramptemp, targettemp, status, segtime):
        # Intentionally no-op: status is derived from the DB by /api/status
        # Keeping the method avoids refactors at call sites.
        return

    # ---- DB helpers (same schema assumptions) ----
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
        self.cur.execute(
            "INSERT INTO Firing (run_id, segment, dt, set_temp, temp, int_temp, pid_output) "
            "VALUES (?, ?, ?, ?, ?, ?, ?)",
            (run_id, seg, time.strftime('%Y-%m-%d %H:%M:%S'), float(set_temp), float(temp), float(int_temp), float(pid_out))
        )
        self.sql.commit()

    # ---- relay autotune (optional) ----
    def maybe_autotune(self, run_id: int, setpoint: float) -> Tuple[float, float, float]:
        if not AUTOTUNE_ON_START:
            return None
        L.info(f"Starting AUTOTUNE for RunID {run_id} at SP={setpoint:.1f}")
        tuner = RelayAutotune(
            setpoint=setpoint,
            read_pv=lambda: self.read_temp()[0],
            write_out=self.hardware.write_output_percent,
            noiseband=AUTOTUNE_NOISE_BAND,
            max_seconds=AUTOTUNE_MAX_SECONDS,
            min_half_cycles=AUTOTUNE_MIN_HALF_CYCLES
        )
        kp, ki, kd = tuner.run()
        L.info(f"AUTOTUNE gains TL: Kp={kp:.5f} Ki={ki:.5f} Kd={kd:.5f}")
        # store to profiles table (p_param/i_param/d_param)
        self.cur.execute("UPDATE profiles SET p_param=?, i_param=?, d_param=? WHERE run_id=?;", (kp, ki, kd, run_id))
        self.sql.commit()
        return kp, ki, kd

    # ---- main segment firing loop (refactor of Fire()) ----
    def fire_segment(self, run_id: int, seg: int, target_tmp: float, rate: float,
                     hold_min: int, window: int, kp: float, ki: float, kd: float):
        """
        Emulates the original segment loop:
        - Construct ramp schedule (RampTmp moving toward TargetTmp).
        - Use PID on RampTmp vs ReadTmp to produce Output% (0..100), with 200 meaning 'error'.
        - Convert Output% to time-proportioning within 'window' seconds.
        """
        L.info(f"Entering segment RunID={run_id} Seg={seg} Target={target_tmp} Rate={rate} HoldMin={hold_min} Window={window}")

        # prepare PID
        self.pid = PIDController(kp, ki, kd, i_min=0.0, i_max=100.0)
        run_state = "Ramp"
        seg_comp_stat = 0

        # read initial temp
        if DEBUG_SIM:
            read_tmp = self.sim_temp
            int_tmp = 25.0
        else:
            read_tmp, int_tmp = self.read_temp()

        last_tmp = 0.0
        last_err = 0.0
        start_tmp = 0.0
        tmp_dif = 0.0
        steps = 0.0
        step_tmp = 0.0
        start_sec = 0
        end_sec = 0
        next_sec = time.time() + window
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

                # fresh reading
                if DEBUG_SIM:
                    try:
                        read_tmp, int_tmp = self.read_temp()
                    except ThermocoupleError:
                        run_state = "Error"
                        break

                # guard bad read
                if math.isnan(read_tmp) or read_tmp > MAX_TEMP_C:
                    read_tmp = last_tmp + last_err
                    print('  "kilntemp": "' + str(int(read_tmp)) + '",\n')

                # initial setup (like original "First pass")
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

                # Rising segment logic
                if tmp_dif > 0:
                    if ramp_trg == 0 and ramp_tmp >= target_tmp:
                        ramp_tmp = target_tmp
                        ramp_trg = 1
                        run_state = "Ramp complete" if read_trg == 0 else "Ramp/Hold"
                    if ((target_tmp - read_tmp) <= 0.5 or read_tmp >= target_tmp) and read_trg == 0:
                        read_trg = 1
                        end_sec = int(time.time()) + hold_min*60
                        L.info(f"Set temp reached - End seconds set to {end_sec}")
                        run_state = "Target Reached" if ramp_trg == 0 else "Target/Hold"

                # Falling segment logic
                elif tmp_dif < 0:
                    if ramp_tmp <= target_tmp and ramp_trg == 0:
                        ramp_tmp = target_tmp
                        ramp_trg = 1
                        run_state = "Ramp complete" if read_trg == 0 else "Target/Ramp"
                    if ((read_tmp - target_tmp) <= 0.5 or read_tmp <= target_tmp) and read_trg == 0:
                        read_trg = 1
                        end_sec = int(time.time()) + hold_min*60
                        L.info(f"Set temp reached - End seconds set to {end_sec}")
                        run_state = "Target Reached" if ramp_trg == 0 else "Ramp/Target"

                # compute PID output against ramp target
                out = self.pid.compute(ramp_tmp, read_tmp)  # 0..100 nominal, 200 signals error
                cycle_on_sec = window * (out * 0.01)
                if cycle_on_sec > window:
                    cycle_on_sec = window

                remain_sec = end_sec - int(time.time())
                rem_min, rem_sec = divmod(max(0, remain_sec), 60)
                rem_hr, rem_min = divmod(rem_min, 60)
                rem_time = f"{rem_hr}:{rem_min:02d}:{rem_sec:02d}"

                L.debug(f"RunID {run_id}, Segment {seg} (loop {cnt}) - RunState:{run_state}, "
                        f"ReadTmp:{read_tmp:.2f}, RampTmp:{ramp_tmp:.2f}, TargetTmp:{target_tmp:.2f}, "
                        f"Output:{out:.2f}, CycleOnSec:{cycle_on_sec:.2f}, RemainTime:{rem_time}")

                # console print for parity with original
                print(f"""RunID {run_id}, Segment {seg} (loop {cnt}) - RunState:{run_state},
                       ReadTmp:{read_tmp:.2f}, RampTmp:{ramp_tmp:.2f}, TargetTmp:{target_tmp:.2f},
                       Output:{out:.2f}, CycleOnSec:{cycle_on_sec:.2f}, RemainTime:{rem_time}
                """)

                # drive relays
                if out > 0:
                    if out == 200.0:
                        # signal error as in original
                        self.cur.execute("UPDATE profiles SET state=? WHERE run_id=?;", ('Error', run_id))
                        self.sql.commit()
                        L.error(f"State = Error RunID: {run_id}")
                        run_state = "Error"
                    else:
                        self.hardware.heat_on()
                        if DEBUG_SIM:
                            self.sim_temp += (cycle_on_sec * 1.0)
                        time.sleep(cycle_on_sec)

                if out < 100.0:
                    self.hardware.heat_off()
                    if DEBUG_SIM:
                        self.sim_temp -= 2.0

                # status file
                self.write_status(read_tmp, run_id, seg, ramp_tmp, target_tmp, run_state, rem_time)

                # Firing row
                try:
                    self.insert_firing_row(run_id, seg, ramp_tmp, read_tmp, int_tmp, out)
                except Exception:
                    self.sql.rollback()
                    L.error("DB insert failed (Firing)")

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

    # ---- main loop (poll for Running profiles, like original) ----
    def run(self):
        L.info("Polling for 'Running' firing profiles...")
        while True:
            # initial status file update (no profile)
            try:
                read_tmp, _ = self.read_temp()
            except ThermocoupleError:
                read_tmp = float('nan')
            self.write_status(read_tmp, None, None, None, None, "n/a", "0:00:00")

            # find running profile
            self.cur.execute("SELECT * FROM profiles WHERE state=?;", ('Running',))
            data = self.cur.fetchall()

            if data:
                run_id = data[0]['run_id']
                kp = float(data[0]['p_param'])
                ki = float(data[0]['i_param'])
                kd = float(data[0]['d_param'])

                # per-run logger
                configure_run_logger(run_id)
                L.info("=== Run start ===")

                # set start time
                try:
                    self.set_profile_start(run_id)
                except Exception:
                    self.sql.rollback()

                # optional autotune (writes gains back)
                try:
                    # pick a reasonable SP to tune around: first segment set_temp
                    self.cur.execute("SELECT set_temp FROM segments WHERE run_id=? ORDER BY segment ASC LIMIT 1;", (run_id,))
                    row = self.cur.fetchone()
                    sp_for_autotune = float(row['set_temp']) if row else 200.0
                    if AUTOTUNE_ON_START:
                        gains = self.maybe_autotune(run_id, sp_for_autotune)
                        if gains is not None:
                            kp, ki, kd = gains
                except Exception as e:
                    L.error(f"Autotune step skipped/failed: {e}")

                # get segments
                self.cur.execute("SELECT * FROM segments WHERE run_id=?;", (run_id,))
                profsegs = self.cur.fetchall()
                total_seg = len(profsegs)
                L.info(f"TotalSeg: {total_seg}")

                run_state_final = None

                for row in profsegs:
                    seg = row['segment']
                    target = row['set_temp']
                    rate = row['rate']
                    hold_min = row['hold_min']
                    window = row['int_sec']

                    # if already finished, skip; else fire
                    if row['start_time'] is not None and row['end_time'] is not None:
                        L.info(f"segment {seg} already finished")
                        continue

                    # mark start
                    try:
                        self.set_segment_start(run_id, seg)
                    except Exception:
                        self.sql.rollback()

                    # execute segment
                    state = self.fire_segment(run_id, seg, target, rate, hold_min, window, kp, ki, kd)
                    run_state_final = state


                    # mark segment end
                    try:
                        self.set_segment_end(run_id, seg)
                    except Exception:
                        self.sql.rollback()

                    # if error/stop, break early
                    if state in ("Error", "Stopped"):
                        break

                # if finished all segments without error/stop -> Completed
                if run_state_final not in ("Error", "Stopped"):
                    try:
                        self.set_profile_end(run_id, 'Completed')
                        L.info("state updated to Completed")
                    except Exception:
                        self.sql.rollback()
                        L.error("DB Update failed (set Completed)")

                # remove run-specific handlers
                configure_run_logger(None)
                L.info("Polling for 'Running' firing profiles...")

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

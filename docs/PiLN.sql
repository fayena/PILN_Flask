--
-- Database: PiLN
--
CREATE TABLE IF NOT EXISTS profiles(
  run_id  INTEGER PRIMARY KEY,
  state   TEXT NOT NULL DEFAULT 'Staged',
  notes   TEXT,
  p_param REAL NOT NULL,
  i_param REAL NOT NULL,
  d_param REAL NOT NULL,
  start_time DATETIME DEFAULT NULL,
  end_time   DATETIME DEFAULT NULL
);
CREATE TABLE IF NOT EXISTS firing(
  run_id     INT NOT NULL,
  segment    INT NOT NULL DEFAULT 0,
  dt         DATETIME NOT NULL DEFAULT CURRENT_DATETIME,
  set_temp   NUMERIC NOT NULL,
  temp       NUMERIC NOT NULL,
  int_temp   NUMERIC DEFAULT NULL,
  pid_output NUMERIC NOT NULL,
  PRIMARY KEY(run_id,segment,dt)
);
CREATE TABLE IF NOT EXISTS segments(
  run_id   INT NOT NULL,
  segment  INT NOT NULL,
  set_temp INT NOT NULL,
  rate     INT NOT NULL,
  hold_min INT NOT NULL,
  int_sec  INT NOT NULL,
  start_time DATETIME DEFAULT NULL,
  end_time   DATETIME DEFAULT NULL,
  PRIMARY KEY (run_id,segment),
  FOREIGN KEY(run_id) REFERENCES profiles(run_id)
);

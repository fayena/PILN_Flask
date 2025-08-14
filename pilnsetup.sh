#!/bin/bash
set -euo pipefail

# ---------- config ----------
REPO_URL="https://github.com/fayena/PILN.git"
APP_HOME="/home/pi/PILN"
VENV_DIR="$APP_HOME/.venv"
SERVICE_FILE="/lib/systemd/system/pilnfired.service"
ENTRYPOINT="$APP_HOME/daemon/pilnfired.py"  # update if different
THERMO_TYPE="S"                        # S, K, etc. Used as environment var

# ---------- OS packages ----------
sudo apt update
sudo apt -y upgrade
sudo apt -y install \
  git sqlite3 ufw lighttpd \
  python3 python3-venv python3-pip \
  python3-jinja2 python3-psutil

# ---------- clone repo (idempotent) ----------
if [ ! -d "$APP_HOME/.git" ]; then
 git clone --depth=1  "$REPO_URL" "$APP_HOME"
else
  cd "$APP_HOME"
  git fetch origin "$REPO_BRANCH"
  git checkout "$REPO_BRANCH"
  git pull
fi

# ---------- create basic dirs ----------
mkdir -p "$APP_HOME/log" "$APP_HOME/style/css" "$APP_HOME/style/js"

# ---------- web assets (same as your script) ----------
wget -q -N -P "$APP_HOME/style/js" https://cdn.datatables.net/1.10.25/js/jquery.dataTables.min.js
wget -q -N -P "$APP_HOME/style/js" https://code.jquery.com/jquery-3.6.0.min.js
wget -q -N -P "$APP_HOME/style/js" https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.5.0/chart.min.js
wget -q -N -P "$APP_HOME/style/js" https://momentjs.com/downloads/moment.min.js
wget -q -N -P "$APP_HOME/style/css" https://cdn.datatables.net/1.10.25/css/jquery.dataTables.min.css

# ---------- lighttpd ----------
sudo cp "$APP_HOME/lighttpd/lighttpd.conf" /etc/lighttpd/
sudo ln -sf ../conf-available/10-cgi.conf /etc/lighttpd/conf-enabled/10-cgi.conf
sudo service lighttpd restart

# ---------- firewall ----------
sudo ufw allow ssh
sudo ufw allow http

# ---------- permissions ----------
sudo chown -R -L www-data:www-data "$APP_HOME/style"
sudo chown pi:pi "$APP_HOME/app/pilnstat.json" || true
sudo chown pi:pi "$APP_HOME/log"
sudo chown -R www-data:www-data "$APP_HOME/db"
sudo touch "$APP_HOME/app/data.json"
sudo chown www-data:www-data "$APP_HOME/app/data.json"

# ---------- enable SPI/I2C (non-interactive if raspi-config is present) ----------
if command -v raspi-config >/dev/null 2>&1; then
  sudo raspi-config nonint do_spi 0 || true
  sudo raspi-config nonint do_i2c 0 || true
else
  # Fallback: ensure dtparams are present (Pi OS Bookworm uses /boot/firmware)
  BOOTCFG="/boot/firmware/config.txt"
  [ -f /boot/config.txt ] && BOOTCFG="/boot/config.txt"
  sudo sed -i -e '/^dtparam=spi=on/!s|^#\?dtparam=spi=.*$||' "$BOOTCFG" || true
  sudo sed -i -e '/^dtparam=i2c_arm=on/!s|^#\?dtparam=i2c_arm=.*$||' "$BOOTCFG" || true
  grep -q '^dtparam=spi=on' "$BOOTCFG" || echo 'dtparam=spi=on' | sudo tee -a "$BOOTCFG"
  grep -q '^dtparam=i2c_arm=on' "$BOOTCFG" || echo 'dtparam=i2c_arm=on' | sudo tee -a "$BOOTCFG"
fi

# Allow the 'pi' user access to SPI/GPIO/I2C devices when not running as root
sudo usermod -aG spi,i2c,gpio pi

# ---------- python venv ----------
python3 -m venv "$VENV_DIR"
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
pip install --upgrade pip wheel setuptools

# Use repo requirements if present, else install minimal set needed by the controller
if [ -f "$APP_HOME/requirements.txt" ]; then
  pip install -r "$APP_HOME/requirements.txt"
else
  pip install \
    adafruit-blinka \
    adafruit-circuitpython-max31856 \
    RPi.GPIO \
    psutil \
    Jinja2
fi
deactivate

# ---------- systemd service ----------
# This rewrites the service to use the venv python and sets env vars.
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=PiLN Kiln Firing Daemon
After=network-online.target lighttpd.service
Wants=network-online.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=$APP_HOME
Environment=THERMOCOUPLE=$THERMO_TYPE
# set SIMULATE_TC=1 to run without hardware (optional)
# Environment=SIMULATE_TC=1
ExecStart=$VENV_DIR/bin/python $ENTRYPOINT
Restart=always
RestartSec=3

# Access to SPI/I2C/GPIO without running as root (user added to groups above)
AmbientCapabilities=CAP_SYS_TTY_CONFIG

[Install]
WantedBy=multi-user.target
EOF

sudo chmod 644 "$SERVICE_FILE"
sudo systemctl daemon-reload
sudo systemctl enable pilnfired.service
sudo systemctl restart pilnfired.service

echo "---------------------------------------"
echo "Install complete."
echo "Service status:"
systemctl --no-pager --full status pilnfired.service || true
sudo systemctl enable pilnfired.service
sudo systemctl start pilnfired.service

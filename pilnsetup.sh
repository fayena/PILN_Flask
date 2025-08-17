#!/bin/bash
set -euo pipefail

# ---------- config ----------
REPO_URL="https://github.com/fayena/PILN_Flask.git"
APP_HOME="/home/pi/PILN"
VENV_DIR="$APP_HOME/.venv"
SERVICE_FILE="/lib/systemd/system/pilnfired.service"
FLASK_SERVICE="/lib/systemd/system/pilnapi.service"
ENTRYPOINT="$APP_HOME/daemon/pilnfired.py"  # update if different
THERMO_TYPE="S"                        # S, K, etc. Used as environment var

# ---------- OS packages ----------
sudo apt update
sudo apt -y upgrade
sudo apt -y install \
  git sqlite3 ufw nginx \
  python3 python3-venv python3-pip \
  python3-jinja2 python3-psutil

# ---------- clone repo (idempotent) ----------
if [ ! -d "$APP_HOME/.git" ]; then
 git clone --depth=1 "$REPO_URL" "$APP_HOME"
else
  echo "git exists"
fi

# ---------- create basic dirs ----------
mkdir -p "$APP_HOME/log" "$APP_HOME/style/css" "$APP_HOME/style/js"

# ---------- web assets (same as your script) ----------
wget -q -N -P "$APP_HOME/style/js" https://cdn.datatables.net/1.10.25/js/jquery.dataTables.min.js
wget -q -N -P "$APP_HOME/style/js" https://code.jquery.com/jquery-3.6.0.min.js
wget -q -N -P "$APP_HOME/style/js" https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.5.0/chart.min.js
wget -q -N -P "$APP_HOME/style/js" https://momentjs.com/downloads/moment.min.js
wget -q -N -P "$APP_HOME/style/css" https://cdn.datatables.net/1.10.25/css/jquery.dataTables.min.css

# ---------- nginx ----------
sudo tee /etc/nginx/sites-available/piln >/dev/null <<'NGINX'
server {
    listen 80;
    server_name _;

    # your project root; so /style maps to /home/pi/PILN/style, /images -> .../images
    root /home/pi/PILN;

    # serve static directly
    location ^~ /style/  { try_files $uri =404; }
    location ^~ /images/ { try_files $uri =404; }
    location = /favicon.ico { try_files $uri =404; }

    # everything else falls back to Flask on :5000
    location / {
        try_files $uri $uri/ @flask;
    }

    location @flask {
        proxy_pass http://127.0.0.1:5000;
        proxy_set_header Host              $host;
        proxy_set_header X-Forwarded-For   $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_set_header X-Forwarded-Host  $host;
        proxy_read_timeout 90s;
    }

    # (optional) longer cache for static
    location ~* \.(css|js|png|jpg|jpeg|gif|ico|svg)$ {
        try_files $uri =404;
        access_log off;
        expires 30d;
        add_header Cache-Control "public";
    }
}
NGINX

sudo ln -sf /etc/nginx/sites-available/piln /etc/nginx/sites-enabled/piln
sudo rm -f /etc/nginx/sites-enabled/default
sudo nginx -t
sudo systemctl reload nginx


# ---------- firewall ----------
sudo ufw allow ssh
sudo ufw allow http
sudo ufw allow 5000/tcp

# ---------- permissions ----------
sudo chown -R -L www-data:www-data "$APP_HOME/style"
sudo chown pi:pi "$APP_HOME/log"
sudo chown -R pi:www-data "$APP_HOME/db"
sudo chmod o+x "$APP_HOME"
sudo chmod 2775 "$APP_HOME/db"         # setgid bit so new files keep group www-data
sudo chmod 664 "$APP_HOME/db/PiLN.sqlite3"

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
    Jinja2 \
    Flask \
    gunicorn
fi
deactivate

# ---------- systemd service ----------
# This rewrites the service to use the venv python and sets env vars.
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=PiLN Kiln Firing Daemon
After=network-online.target nginx.service
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
UMask=0002

# Access to SPI/I2C/GPIO without running as root (user added to groups above)
AmbientCapabilities=CAP_SYS_TTY_CONFIG

[Install]
WantedBy=multi-user.target
EOF

sudo chmod 644 "$SERVICE_FILE"
sudo systemctl daemon-reload
sudo systemctl enable pilnfired.service
sudo systemctl restart pilnfired.service

# ---------- flask service ----------
# Adds a flask service
sudo tee "$FLASK_SERVICE" >/dev/null <<EOF
[Unit]
Description=PiLN Flask API (Gunicorn)
After=network-online.target
Wants=network-online.target

[Service]
User=pi
Group=pi
WorkingDirectory=/home/pi/PILN
Environment=PYTHONUNBUFFERED=1
ExecStart=/home/pi/PILN/.venv/bin/gunicorn -b 127.0.0.1:5000 -w 2 api.app:app
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable pilnapi.service
sudo systemctl start pilnapi.service
sudo systemctl status pilnapi.service --no-pager

echo "---------------------------------------"
echo "Install complete."
echo "Service status:"
systemctl --no-pager --full status pilnfired.service || true
sudo systemctl enable pilnfired.service
sudo systemctl start pilnfired.service

#!/bin/bash
set -euo pipefail

# ============================================================
#                     CONFIG (EDIT ME)
#   Set these ONCE and they propagate to:
#   - repo location
#   - piln.env (generated)
#   - systemd units
#   - nginx site
# ============================================================

REPO_URL="https://github.com/fayena/PILN_Flask.git"

# App user (Pi OS now asks during install; don't assume 'pi')
APP_USER="alforddm"
APP_GROUP="$APP_USER"

# App location
USER_HOME="/home/${APP_USER}"
APP_NAME="PILN"
APP_HOME="${USER_HOME}/${APP_NAME}"

# Python / entrypoints
VENV_DIR="${APP_HOME}/.venv"
ENTRYPOINT="${APP_HOME}/daemon/pilnfired.py"

# DB
DB_DIR="${APP_HOME}/db"
DB_FILE="${DB_DIR}/PiLN.sqlite3"

# Generated env file (stored in the repo folder)
ENV_FILE="${APP_HOME}/piln.env"

# systemd
SYSTEMD_DIR="/lib/systemd/system"
FIRE_SERVICE_NAME="pilnfired.service"
API_SERVICE_NAME="pilnapi.service"
FIRE_SERVICE_FILE="${SYSTEMD_DIR}/${FIRE_SERVICE_NAME}"
API_SERVICE_FILE="${SYSTEMD_DIR}/${API_SERVICE_NAME}"

# gunicorn / flask
GUNICORN_BIND="127.0.0.1:5000"
GUNICORN_WORKERS=2
FLASK_APP_MODULE="api.app:app"

# nginx (Option A: serve static from a dedicated web root)
NGINX_SITE_NAME="piln"
NGINX_AVAILABLE="/etc/nginx/sites-available/${NGINX_SITE_NAME}"
NGINX_ENABLED="/etc/nginx/sites-enabled/${NGINX_SITE_NAME}"
WEB_ROOT="/var/www/piln"   # nginx will NOT need access to /home/$APP_USER

# hardware
THERMO_TYPE="S"            # S, K, etc.

# Optional override (usually leave empty to auto-detect from OS)
# PILN_TIMEZONE_OVERRIDE="UTC"
PILN_TIMEZONE_OVERRIDE=""

# ============================================================
#         AUTO-DETECT OS TIMEZONE (systemd timedatectl)
#   - User doesn't need to set it manually.
#   - If PILN_TIMEZONE_OVERRIDE is set, that wins.
# ============================================================

PILN_TIMEZONE=""
if [ -n "${PILN_TIMEZONE_OVERRIDE}" ]; then
  PILN_TIMEZONE="${PILN_TIMEZONE_OVERRIDE}"
else
  PILN_TIMEZONE="$(
    timedatectl show -p Timezone --value 2>/dev/null \
    || true
  )"
  if [ -z "${PILN_TIMEZONE}" ] && [ -f /etc/timezone ]; then
    PILN_TIMEZONE="$(tr -d ' \t\r\n' </etc/timezone)"
  fi
  PILN_TIMEZONE="${PILN_TIMEZONE:-UTC}"
fi

# ============================================================
#                     OS PACKAGES
# ============================================================

sudo apt update
sudo apt -y upgrade
sudo apt -y install \
  git sqlite3 ufw nginx rsync \
  python3 python3-venv python3-pip \
  python3-jinja2 python3-psutil

# ============================================================
#                     CLONE REPO
# ============================================================

# Ensure parent dir exists and is owned by APP_USER
sudo mkdir -p "${APP_HOME%/*}"
sudo chown -R "${APP_USER}:${APP_GROUP}" "${APP_HOME%/*}"

if [ ! -d "${APP_HOME}/.git" ]; then
  sudo -u "${APP_USER}" git clone --depth=1 "${REPO_URL}" "${APP_HOME}"
else
  echo "Repo already exists at ${APP_HOME}"
fi

# ============================================================
#                     DIRECTORIES
# ============================================================

sudo -u "${APP_USER}" mkdir -p \
  "${APP_HOME}/log" \
  "${APP_HOME}/style/css" \
  "${APP_HOME}/style/js" \
  "${DB_DIR}"

sudo mkdir -p "${WEB_ROOT}"

# ============================================================
#                     WEB ASSETS (download into repo)
# ============================================================

sudo -u "${APP_USER}" wget -q -N -P "${APP_HOME}/style/js"  https://cdn.datatables.net/1.10.25/js/jquery.dataTables.min.js
sudo -u "${APP_USER}" wget -q -N -P "${APP_HOME}/style/js"  https://code.jquery.com/jquery-3.6.0.min.js
sudo -u "${APP_USER}" wget -q -N -P "${APP_HOME}/style/js"  https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.5.0/chart.min.js
sudo -u "${APP_USER}" wget -q -N -P "${APP_HOME}/style/js"  https://momentjs.com/downloads/moment.min.js
sudo -u "${APP_USER}" wget -q -N -P "${APP_HOME}/style/css" https://cdn.datatables.net/1.10.25/css/jquery.dataTables.min.css

# ============================================================
#                     DEPLOY STATIC TO WEB_ROOT
#   nginx serves /style and /images from WEB_ROOT
# ============================================================

sudo rsync -a --delete "${APP_HOME}/style/" "${WEB_ROOT}/style/"
if [ -d "${APP_HOME}/images" ]; then
  sudo rsync -a --delete "${APP_HOME}/images/" "${WEB_ROOT}/images/"
fi

sudo chown -R www-data:www-data "${WEB_ROOT}"
sudo chmod -R u=rwX,g=rX,o=rX "${WEB_ROOT}"
sudo find "${WEB_ROOT}" -type d -exec chmod 755 {} \;
sudo find "${WEB_ROOT}" -type f -exec chmod 644 {} \; || true

# ============================================================
#                     NGINX
#   - root is WEB_ROOT (not APP_HOME)
#   - static served directly
#   - everything else proxies to gunicorn
# ============================================================

sudo tee "${NGINX_AVAILABLE}" >/dev/null <<NGINX
server {
    listen 80;
    server_name _;

    root ${WEB_ROOT};

    location ^~ /style/  { try_files \$uri =404; }
    location ^~ /images/ { try_files \$uri =404; }
    location = /favicon.ico { try_files \$uri =404; }

    location / {
        try_files \$uri \$uri/ @flask;
    }

    location @flask {
        proxy_pass http://${GUNICORN_BIND};
        proxy_set_header Host              \$host;
        proxy_set_header X-Forwarded-For   \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
        proxy_set_header X-Forwarded-Host  \$host;
        proxy_read_timeout 90s;
    }

    location ~* \.(css|js|png|jpg|jpeg|gif|ico|svg)$ {
        try_files \$uri =404;
        access_log off;
        expires 30d;
        add_header Cache-Control "public";
    }
}
NGINX

sudo ln -sf "${NGINX_AVAILABLE}" "${NGINX_ENABLED}"
sudo rm -f /etc/nginx/sites-enabled/default
sudo nginx -t
sudo systemctl reload nginx

# ============================================================
#                     FIREWALL
# ============================================================

sudo ufw allow ssh
sudo ufw allow http
sudo ufw allow 5000/tcp

# ============================================================
#                     DB PERMISSIONS (IMPORTANT)
#   - services run as APP_USER
#   - sqlite needs to create journal/WAL files in DB_DIR
# ============================================================

sudo mkdir -p "${DB_DIR}"
sudo chown -R "${APP_USER}:www-data" "${DB_DIR}"
sudo chmod 2775 "${DB_DIR}"

if [ ! -f "${DB_FILE}" ]; then
  sudo -u "${APP_USER}" touch "${DB_FILE}"
fi
sudo chmod 664 "${DB_FILE}"

# ============================================================
#                     WRITE piln.env (GENERATED)
#   Stored inside the repo folder so it stays self-contained.
#   systemd services will load this via EnvironmentFile=
# ============================================================

sudo tee "${ENV_FILE}" >/dev/null <<EOF
# Generated by install.sh. Edit only if needed.
PILN_HOME=${APP_HOME}
PILN_DB_PATH=${DB_FILE}
PILN_STAT_FILE=${APP_HOME}/app/pilnstat.json
PILN_TEMPLATE_DIR=${APP_HOME}/template
PILN_TIMEZONE=${PILN_TIMEZONE}

THERMOCOUPLE=${THERMO_TYPE}

# Optional runtime knobs
GUNICORN_BIND=${GUNICORN_BIND}
GUNICORN_WORKERS=${GUNICORN_WORKERS}
FLASK_APP_MODULE=${FLASK_APP_MODULE}
EOF

sudo chown "${APP_USER}:${APP_GROUP}" "${ENV_FILE}"
sudo chmod 644 "${ENV_FILE}"

# ============================================================
#                     ENABLE SPI/I2C
# ============================================================

if command -v raspi-config >/dev/null 2>&1; then
  sudo raspi-config nonint do_spi 0 || true
  sudo raspi-config nonint do_i2c 0 || true
else
  BOOTCFG="/boot/firmware/config.txt"
  [ -f /boot/config.txt ] && BOOTCFG="/boot/config.txt"
  grep -q '^dtparam=spi=on' "$BOOTCFG" || echo 'dtparam=spi=on' | sudo tee -a "$BOOTCFG"
  grep -q '^dtparam=i2c_arm=on' "$BOOTCFG" || echo 'dtparam=i2c_arm=on' | sudo tee -a "$BOOTCFG"
fi

sudo usermod -aG spi,i2c,gpio "${APP_USER}"

# ============================================================
#                     PYTHON VENV
# ============================================================

sudo -u "${APP_USER}" python3 -m venv "${VENV_DIR}"
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
pip install --upgrade pip wheel setuptools

if [ -f "${APP_HOME}/requirements.txt" ]; then
  pip install -r "${APP_HOME}/requirements.txt"
else
  pip install \
    adafruit-blinka \
    adafruit-circuitpython-max31856 \
    RPi.GPIO \
    psutil \
    Jinja2 \
    Flask \
    gunicorn \
    numpy \
    scipy
fi
deactivate

# ============================================================
#                     SYSTEMD: pilnfired.service
#   Loads env vars from ENV_FILE (in repo)
# ============================================================

sudo tee "${FIRE_SERVICE_FILE}" >/dev/null <<EOF
[Unit]
Description=PiLN Kiln Firing Daemon
After=network-online.target nginx.service
Wants=network-online.target

[Service]
Type=simple
User=${APP_USER}
Group=${APP_GROUP}
WorkingDirectory=${APP_HOME}
EnvironmentFile=${ENV_FILE}
ExecStart=${VENV_DIR}/bin/python ${ENTRYPOINT}
Restart=always
RestartSec=3
UMask=0002
AmbientCapabilities=CAP_SYS_TTY_CONFIG

[Install]
WantedBy=multi-user.target
EOF

# ============================================================
#                     SYSTEMD: pilnapi.service
#   Loads env vars from ENV_FILE (in repo)
# ============================================================

sudo tee "${API_SERVICE_FILE}" >/dev/null <<EOF
[Unit]
Description=PiLN Flask API (Gunicorn)
After=network-online.target
Wants=network-online.target

[Service]
User=${APP_USER}
Group=${APP_GROUP}
WorkingDirectory=${APP_HOME}
EnvironmentFile=${ENV_FILE}
Environment=PYTHONUNBUFFERED=1
ExecStart=${VENV_DIR}/bin/gunicorn -b ${GUNICORN_BIND} -w ${GUNICORN_WORKERS} ${FLASK_APP_MODULE}
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "${FIRE_SERVICE_NAME}" "${API_SERVICE_NAME}"
sudo systemctl restart "${FIRE_SERVICE_NAME}" "${API_SERVICE_NAME}"

# ============================================================
#                     OPTIONAL: sync static on boot
# ============================================================

STATIC_SYNC_NAME="pilnstatic-sync.service"
STATIC_SYNC_FILE="${SYSTEMD_DIR}/${STATIC_SYNC_NAME}"

sudo tee "${STATIC_SYNC_FILE}" >/dev/null <<EOF
[Unit]
Description=PiLN Static Assets Sync to ${WEB_ROOT}
After=network-online.target

[Service]
Type=oneshot
ExecStart=/usr/bin/rsync -a --delete ${APP_HOME}/style/ ${WEB_ROOT}/style/
ExecStart=/bin/bash -lc 'if [ -d "${APP_HOME}/images" ]; then /usr/bin/rsync -a --delete ${APP_HOME}/images/ ${WEB_ROOT}/images/; fi'
ExecStart=/bin/bash -lc '/bin/chown -R www-data:www-data ${WEB_ROOT} && /bin/chmod -R u=rwX,g=rX,o=rX ${WEB_ROOT}'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "${STATIC_SYNC_NAME}"
sudo systemctl start "${STATIC_SYNC_NAME}"

echo "---------------------------------------"
echo "Install complete."
echo "OS timezone detected as: ${PILN_TIMEZONE}"
echo "Env file: ${ENV_FILE}"
echo "Nginx root: ${WEB_ROOT}"
echo "Services:"
systemctl --no-pager --full status "${API_SERVICE_NAME}" || true
systemctl --no-pager --full status "${FIRE_SERVICE_NAME}" || true

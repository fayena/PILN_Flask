# /home/pi/PILN/api/app.py
from flask import Flask, jsonify
import json
import os

APP_DIR = "/home/pi/PILN"
STAT_FILE = os.path.join(APP_DIR, "app", "pilnstat.json")

app = Flask(__name__)

@app.route("/api/health")
def health():
    return jsonify({"ok": True})

@app.route("/api/status")
def status():
    try:
        with open(STAT_FILE) as f:
            data = json.load(f)
        return jsonify(data)
    except FileNotFoundError:
        return jsonify({"error": "status file not found"}), 404

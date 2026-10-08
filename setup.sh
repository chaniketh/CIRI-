#!/bin/sh
set -eu
cd "$(dirname "$0")"
[ -f config.json ] || cp config.example.json config.json
[ -f calibration.uno.json ] || cp calibration.example.json calibration.uno.json
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
printf '\nSetup complete. Run: ./start.sh\n'

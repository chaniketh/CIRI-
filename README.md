# CIRI — Cat inspired Retractable interlocking

## Needle force logger

Arduino Uno + HX711 load cell + USB Dynamixel. A local web page records force, material, needle angle, insertion level, and test outcome.

The local working copy retains your rig configuration and calibration. GitHub includes templates; on a new computer, enter your verified calibration and motor port before a real test. Default pull: **10 seconds at about 15 rpm** (15.114 rpm in motor units), toward the motor. Force and angle stops are disabled. Sensor faults, USB faults, browser disconnection, and the motor watchdog still stop the test. Allow enough cart clearance for about 2.5 motor revolutions; disabling software limits does not increase the load cell's capacity.

## 1. Install once

On macOS, install Python 3 if `python3 --version` does not work. Open Terminal in this repository:

```sh
git clone https://github.com/chaniketh/CIRI-.git
cd CIRI-
./setup.sh
```

`setup.sh` runs these commands:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

No activation is required. Dependencies: pyserial, Dynamixel SDK, matplotlib.

## 2. Connect the hardware

- HX711 VCC → Uno 5V; GND → GND; DT/DOUT → D2; SCK → D3.
- Load-cell bridge → HX711 E+, E−, A+, A− according to the sensor wiring. Wire colors are not universal.
- Uno → computer USB. Keep its cable slack as the cart moves.
- Dynamixel → USB control board, with its separate motor power supply connected.
- Current motor: **XL430-W250, model 1060, ID 1, Protocol 2.0, baud 57600**. Supported alternate profile: XM430-W350, model 1020. Firmware must be at least 38 for this controller.

In Arduino IDE:

1. Install **HX711 by Bogdan Necula / bogde** in Library Manager.
2. Select **Arduino Uno** and its port.
3. Open `uno_load_cell/uno_load_cell.ino` and upload it.
4. Close Serial Monitor before using the logger.

The normal sketch streams `sequence,uno_ms,raw_counts,saturated` at 115200 baud.

## 3. Check ports and configuration

```sh
.venv/bin/python -m serial.tools.list_ports -v
```

The GUI detects the Uno automatically by USB identity, including changes to its `/dev/cu.usbmodem…` path. With one compatible board attached it is selected automatically. To select a particular Uno, add `uno_serial_number` to `config.json` using the port-list output. With multiple boards, use the correct serial number.

The Dynamixel port is configured separately in `config.json` under `motor.port`. Use the device path shown by the port-list command. If it changes, update it before starting the server. Close Dynamixel Wizard to release that port.

Optional read-only motor check:

```sh
.venv/bin/python identify_motor.py --port /dev/cu.usbserial_REPLACE --id 1 --baud 57600
```

The saved direction is verified for this rig. If changing the drive or mounting, verify it unloaded before pulling:

```sh
.venv/bin/python direction_check.py --config config.json --direction cw
```

That command moves the motor slowly for a short jog. Set `cw_sign` and `direction_verified` in `config.json` only after confirming the intended cart direction.

## 4. Start and log

```sh
./start.sh
```

Equivalent command:

```sh
.venv/bin/python web_gui.py --config config.json
```

Open **http://127.0.0.1:8765/**. Keep Terminal and the page open.

1. Enter material, needle angle **from the horizontal rail**, and insertion level: on surface / in surface / deep into surface.
2. With the needle clear of material, click **Tare sensor**. Wait for Ready.
3. Engage the material. Check cart clearance, then click **Start test**.
4. The page records force and peak magnitude. The − / + buttons change speed; changes during a run are logged.
5. After the run, select the outcome, add any notes, and click **Finish test & save**. Clicking Finish during a run stops it early.
6. Tare again before the next test.

Each test is saved automatically in `runs/<timestamp>/`:

- `samples.csv`: timestamped signed force in N, raw counts, trial labels, angle, motor position and speed.
- `metadata.json`: configuration, session tare, calibration, peak force, outcome, notes, and stop reason.

Use the ZIP download in the page's Test log to share a test. Negative force indicates the calibrated sensor sign; use its magnitude for peak comparisons. The logger leaves calculated needle-force components blank while the force model is unconfirmed.

Stop the server with **Ctrl+C** in Terminal. Run `./start.sh` again to restart it. Do not run two servers/loggers against the same hardware.

## 5. Make a graph

Replace the timestamp below with the test folder you want:

```sh
.venv/bin/python analyze.py runs/YOUR_TEST_TIMESTAMP --plot
```

This writes analysis files and a force plot into that test folder. To list tests:

```sh
ls runs
```

## Calibration

Your local rig calibration is `calibration.uno.json`; it is excluded from GitHub. A fresh setup copies `calibration.example.json` to that filename. Fill its constants using the procedure below before running a real test. Every Tare sets a fresh unloaded zero for that test.

To recalibrate, upload `uno_calibrate/uno_calibrate.ino` in Arduino IDE. Open Serial Monitor at **115200 baud**, with **Newline** selected. Apply the known loads at the same needle contact point and in the test force direction. Include the hanger's mass. A weight placed elsewhere on the beam does not establish the needle-contact calibration.

Send these commands one at a time, changing the load between commands:

```text
t        (unloaded)
c 50     (actual total mass of 50 g applied)
v 100    (different actual total mass of 100 g applied)
p        (print verified constants)
```

Substitute your actual measured masses. Check return to zero after unloading. Copy the printed `COUNTS_PER_NEWTON`, `COUNTS_PER_GRAM`, and `OFFSET_COUNTS` into `counts_per_N`, `counts_per_gram`, and `offset_counts` in `calibration.uno.json`.

**Re-upload `uno_load_cell/uno_load_cell.ino` afterward**, then close Serial Monitor. The calibration sketch does not produce the CSV packets needed by the GUI.

## Common fixes

- **Site cannot be reached:** run `./start.sh`, leave Terminal open, then refresh the page.
- **Resource busy:** close Arduino Serial Monitor, Dynamixel Wizard, and other loggers.
- **Uno port changed:** the GUI detects it automatically. Click Tare to connect.
- **Device not configured / USB disconnected during a test:** finish the partial log, secure/reconnect USB, then tare and start a new test. It never resumes motion automatically.
- **HX711 not ready / no samples:** check wiring and upload the normal `uno_load_cell` sketch.
- **Stopped early:** check the saved stop reason. Faults still end the test even with force and angle thresholds disabled.

## Check the software without moving hardware

```sh
.venv/bin/python -m unittest discover -s tests -q
./start.sh --config config.demo.json --demo --port 8766
```

Open http://127.0.0.1:8766/ for simulated data. Demo logs are separate from real tests.

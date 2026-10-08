# CIRI — Cat inspired Retractable interlocking

## Needle force logger

Arduino Uno + HX711 load cell + USB Dynamixel. A local web page records force, material, needle angle, insertion level, and test outcome.

The setup includes your existing verified load-cell calibration. Recalibration is optional for the unchanged rig; set the motor port and verify its direction on the new computer. Default pull: **10 seconds at about 15 rpm** (15.114 rpm in motor units), toward the motor. Force and angle stops are disabled. Sensor faults, USB faults, browser disconnection, and the motor watchdog still stop the test. Allow enough cart clearance for about 2.5 motor revolutions; disabling software limits does not increase the load cell's capacity.

## 1. Install once

### macOS / Linux

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

### Windows (PowerShell)

Install Python 3, Git, and Arduino IDE. Open PowerShell and run:

```powershell
git clone https://github.com/chaniketh/CIRI-.git
cd CIRI-
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
if (!(Test-Path config.json)) { Copy-Item config.example.json config.json }
if (!(Test-Path calibration.uno.json)) { Copy-Item calibration.example.json calibration.uno.json }
```

No environment activation or PowerShell execution-policy change is needed. The `.sh` scripts are for macOS/Linux; on Windows use the Python commands below. If `py` is unavailable but `python --version` shows Python 3, use `python -m venv .venv` instead.

The setup copies the supplied calibration constants automatically. Before a real test, set the motor COM port and verify motor direction in `config.json`. No calibration weights are needed for normal logging.

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

macOS/Linux:

```sh
.venv/bin/python -m serial.tools.list_ports -v
```

Windows:

```powershell
.\.venv\Scripts\python.exe -m serial.tools.list_ports -v
```

The GUI detects the Uno automatically by USB identity, including changes to its `/dev/cu.usbmodem…` path on macOS or `COM` number on Windows. With one compatible board attached it is selected automatically. To select a particular Uno, add `uno_serial_number` to `config.json` using the port-list output. With multiple boards, use the correct serial number.

The Dynamixel port is configured separately in `config.json` under `motor.port`. Use the device path shown by the port-list command. If it changes, update it before starting the server. Close Dynamixel Wizard to release that port. On Windows, use a value such as `"COM5"` (replace it with the actual motor adapter port). The Uno and motor must have different ports. If either device is missing, check Windows Device Manager and install the manufacturer’s USB driver if needed.

Optional read-only motor check:

```sh
.venv/bin/python identify_motor.py --port /dev/cu.usbserial_REPLACE --id 1 --baud 57600
```

On Windows, the same read-only check is:

```powershell
.\.venv\Scripts\python.exe identify_motor.py --port COM5 --id 1 --baud 57600
```

Replace `COM5` with your actual motor port. The public template sets `direction_verified` to false. Verify direction unloaded before pulling:

```sh
.venv/bin/python direction_check.py --config config.json --direction cw
```

Windows unloaded jog:

```powershell
.\.venv\Scripts\python.exe direction_check.py --config config.json --direction cw
```

That command moves the motor slowly for a short jog. Set `cw_sign` and `direction_verified` in `config.json` only after confirming the intended cart direction.

## 4. Start and log

macOS/Linux:

```sh
./start.sh
```

Equivalent command:

```sh
.venv/bin/python web_gui.py --config config.json
```

Windows:

```powershell
.\.venv\Scripts\python.exe web_gui.py --config config.json
```

Open **http://127.0.0.1:8765/**. Keep Terminal/PowerShell and the page open.

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

Stop the server with **Ctrl+C** in Terminal. Run `./start.sh` on macOS/Linux, or the Windows `web_gui.py` command above, to restart it. Do not run two servers/loggers against the same hardware.

## 5. Make a graph

Replace the timestamp below with the test folder you want:

```sh
.venv/bin/python analyze.py runs/YOUR_TEST_TIMESTAMP --plot
```

Windows:

```powershell
.\.venv\Scripts\python.exe analyze.py runs\YOUR_TEST_TIMESTAMP --plot
Get-ChildItem runs
```

This writes analysis files and a force plot into that test folder. To list tests:

```sh
ls runs
```

## Existing calibration — ready to use

`calibration.example.json` already contains your supplied verified values:

| Setting | Value |
| --- | ---: |
| `counts_per_N` | 73375.585937 |
| `counts_per_gram` | 719.568664 |
| `offset_counts` | 22706.33 |

The setup commands copy these into `calibration.uno.json`, which both Python loggers use through `config.json`. The Uno logging sketch sends raw counts; conversion to newtons happens in Python:

```text
Force_N = (raw_counts - session_tare_counts) / 73375.585937
```

Click **Tare sensor unloaded before every test**. This replaces the old offset for that run without changing the saved sensitivity. Tare is not a full recalibration. These constants apply to the same sensor, HX711, wiring, mounting, and load path; they do not establish a total needle-contact force for a different fixture geometry.

If you previously copied the blank template, stop the logger and update your local calibration after `git pull`. Preserve any custom calibration first:

Windows:

```powershell
if (Test-Path calibration.uno.json) { Copy-Item calibration.uno.json calibration.backup.json }
Copy-Item calibration.example.json calibration.uno.json
```

macOS/Linux:

```sh
[ ! -f calibration.uno.json ] || cp calibration.uno.json calibration.backup.json
cp calibration.example.json calibration.uno.json
```

## Recalibration (optional)

Skip this section for the unchanged, calibrated rig. Use it if you replace the sensor/HX711, alter the mounting or load path, or verification no longer agrees with a known load.

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

- **Site cannot be reached:** run the start command for your operating system, leave Terminal open, then refresh the page.
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

Windows:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe web_gui.py --config config.demo.json --demo --port 8766
```

Open http://127.0.0.1:8766/ for simulated data. Demo logs are separate from real tests.

Windows hardware operation has not yet been tested. The logger uses cross-platform Python and pyserial COM-port discovery; verify with the demo and an unloaded hardware test first.

# MaixCAM-Pro + ESP32-S3 validation tools

These scripts collect and summarize the `HWCSV` records emitted by the firmware.
They do not generate paper results: a summary is marked `synthetic_demo` when
created with `capture_serial.py --demo` and `physical_serial_capture` otherwise.

```powershell
python -m pip install -r requirements.txt
python generate_trial_manifest.py --actors 8 --repeats 5 --out trial_manifest.csv
python set_trial.py a01_r01_con_HH --port COM9
python capture_serial.py --port COM8 --baud 115200 --duration 300 --out run_01.log
python analyze_logs.py run_01.log --out-dir run_01_analysis
```

Before connecting hardware, verify the parser with:

```powershell
python -m unittest discover -s tests
python capture_serial.py --demo --duration 2 --out demo.log
python analyze_logs.py demo.log --out-dir demo_analysis
```

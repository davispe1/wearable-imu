# tools/

Utility scripts for flashing, data capture, and calibration.

**Status: planned, not started.** Nothing in this folder is implemented yet;
the table below is the intended scope, not an inventory.

| Script (planned) | Purpose |
|--------------|---------|
| `flash.py` | Flash a node via ST-LINK (wraps openocd / STM32CubeProgrammer) |
| `capture.py` | Capture a raw SWO session to file for offline replay |
| `calibrate.py` | Run the T/N-pose calibration wizard and save a profile |
| `replay.py` | Feed a captured session file through the host pipeline |

# Timeout Fix (Simple)

If you see timeout during failure injection, the most common reason is:

- `SYS_FAILURE_EN` is disabled.

## Fix

In PX4 shell:

```bash
param set SYS_FAILURE_EN 1
param save
```

Restart PX4 SITL, then run automated tests:

```bash
cd ~/sky_warriors_ws/src/failsafes/scripts
python3 mavsdkrunner_automated.py
```

## Extra check

If needed, confirm:

```bash
param show SYS_FAILURE_EN
```

You want current value = `1`.

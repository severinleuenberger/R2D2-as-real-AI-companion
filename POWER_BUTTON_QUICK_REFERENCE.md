# R2D2 Power Button - Quick Reference

## Hardware Wiring

```
Momentary Button (O-ring switch)
├─ Terminal A ──→ Pin 22 (GPIO17) on 40-pin header
│               ──→ Pin 2 (PWR) on J42 header [PARALLEL]
│
└─ Terminal B ──→ Pin 20 (GND) on 40-pin header
                ──→ Pin 1 (GND) on J42 header
```

## Installation (One Command)

```bash
cd ~/dev/r2d2 && sudo bash install_power_button.sh
```

## Button Behavior

| Button Action | Result |
|---------------|--------|
| **Single Press** (tap) | Enter low-power mode |
| **Double Press** (2 taps <400ms) | Graceful shutdown |
| **Long Press** (hold 1+s) | Counts as single press |
| **Press When Off** | Wake/boot (hardware) |

## Quick Commands

```bash
# Check service status
sudo systemctl status r2d2-powerbutton.service

# Watch logs in real-time
sudo journalctl -u r2d2-powerbutton.service -f

# Restart service
sudo systemctl restart r2d2-powerbutton.service

# Stop service
sudo systemctl stop r2d2-powerbutton.service

# View last 20 log lines
sudo journalctl -u r2d2-powerbutton.service -n 20
```

## Key Files

| File | Purpose | Location |
|------|---------|----------|
| `r2d2_power_button.py` | Main handler script | `/usr/local/bin/` |
| `r2d2-powerbutton.service` | Systemd service | `/etc/systemd/system/` |
| Logs | Journal output | `journalctl -u r2d2-powerbutton.service` |

## Testing Checklist

- [ ] Installation completes without errors
- [ ] Service is running: `systemctl status r2d2-powerbutton.service`
- [ ] Logs show "Button monitor active"
- [ ] Single press → low-power mode (check with `nvpmodel -q cur`)
- [ ] Double press → shutdown confirmation in logs
- [ ] Power-on from OFF works (via J42 hardware)

## Troubleshooting

**Service won't start:**
```bash
sudo apt install python3-jetson-gpio
sudo systemctl restart r2d2-powerbutton.service
```

**Button doesn't respond:**
```bash
# Verify GPIO access
python3 -c "import Jetson.GPIO; print('GPIO OK')"
```

**View detailed errors:**
```bash
sudo systemctl status r2d2-powerbutton.service
sudo journalctl -u r2d2-powerbutton.service --no-pager
```

## Important Safety Notes

⚠️ Double-press initiates **SHUTDOWN** - be careful!
⚠️ Single press only enters low-power mode
⚠️ Service auto-restarts if it crashes
⚠️ All actions are logged to journalctl

---

**Full Testing Guide:** See `POWER_BUTTON_TESTING_GUIDE.md`

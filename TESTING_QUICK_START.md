# R2D2 Power Button - Testing Quick Start

**Ready to Test?** Start here!

---

## Pre-Test Check (2 minutes)

```bash
# Verify service is running
sudo systemctl status r2d2-powerbutton.service

# Should show: "Active: active (running)"
```

If not running:
```bash
sudo systemctl restart r2d2-powerbutton.service
```

---

## Test 1: Single Press → Low-Power (5 min)

**In Terminal A:**
```bash
journalctl -u r2d2-powerbutton.service -f
```

**Then:**
1. Press **Button 1** (Pin 32) **ONCE** - single short tap
2. Look for log: `"Single press detected"` then `"✓ Low-power mode activated"`
3. System should become quieter (fans reduce speed)

**Result:** ✅ PASS / ❌ FAIL

---

## Test 2: Double Press → Shutdown (10 min)

**In Terminal A:**
```bash
journalctl -u r2d2-powerbutton.service -f
```

**Then:**
1. Press **Button 1** **TWICE** quickly (200-300ms apart)
2. Look for logs: `"Press count: 2"` then `"Double-press confirmed"` then `"Initiating graceful shutdown"`
3. System should shutdown (~10 seconds)

**Result:** ✅ PASS / ❌ FAIL

---

## Test 3: Button 2 Wake (5 min)

**Prerequisite:** System in low-power mode (from Test 1)

**Then:**
1. Press **Button 2** (J42 Pin 2) **ONCE**
2. System should become responsive again
3. CPU fans spin up, services resume

**Result:** ✅ PASS / ❌ FAIL

---

## Test 4: Button 2 Boot (10 min)

**Prerequisite:** System shutdown (from Test 2)

**Then:**
1. Wait 5 seconds for complete shutdown
2. Press **Button 2** (J42 Pin 2) **ONCE**
3. System should boot up normally (~30-60 seconds)

**Result:** ✅ PASS / ❌ FAIL

---

## Summary

| Test | Status |
|------|--------|
| Single Press → Low-Power | [ ] PASS / [ ] FAIL |
| Double Press → Shutdown | [ ] PASS / [ ] FAIL |
| Button 2 → Wake | [ ] PASS / [ ] FAIL |
| Button 2 → Boot | [ ] PASS / [ ] FAIL |

---

## Troubleshooting

**Nothing happens when I press?**
- Check button is wired correctly
- Run: `gpioget gpiochip0 106` to test GPIO directly
- Check logs: `journalctl -u r2d2-powerbutton.service -n 50`

**Wrong action triggered?**
- Double-press timing might be off (try faster/slower)
- Check if first press was detected in logs
- Timing window is now 1000ms (very forgiving)

**System unresponsive?**
- SSH in: `ssh severin@R2D2`
- Check service: `sudo systemctl status r2d2-powerbutton.service`
- Restart: `sudo systemctl restart r2d2-powerbutton.service`

**Need full logs?**
- All logs: `cat /var/log/r2d2_power_button.log`
- Recent logs: `journalctl -u r2d2-powerbutton.service -n 100`
- Live logs: `journalctl -u r2d2-powerbutton.service -f`

---

## Full Documentation

For detailed test procedures, see: `POWER_BUTTON_TEST_PLAN.md`

For complete setup guide, see: `POWER_BUTTON_FINAL_SETUP.md`

---

**Good luck with testing! 🚀**

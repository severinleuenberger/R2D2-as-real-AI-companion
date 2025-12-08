# J511 Audio Header - CORRECT PINOUT

**Source:** Jetson Nano Carrier Board Documentation (Official)

## J511 Header Pin Layout

```
TOP VIEW (Looking down at the board):

Pin 1 marker (triangle/square) is at top-left

        ╔═══════════════════════════╗
Row A:  ║ 1    3    5    7    9     ║
        ║                            ║
Row B:  ║ 2    4    6    8   10     ║
        ╚═══════════════════════════╝
```

## Pin Assignments (from official documentation)

| Pin | Signal Name | Type | Function |
|-----|-------------|------|----------|
| 1 | IN1P | Input | Microphone #1 input |
| 2 | AGND | GND | **Audio Ground** |
| 3 | IN2P | Input | Microphone #2 input |
| 4 | LRCK2/GPIO4/PDM_SDA | Input | Audio dongle detection |
| **5** | **HPO_R** | **Output** | **Headphone RIGHT channel** |
| 6 | MIC_IN_DET | Input | Jack/Microphone detect |
| 7 | SENSE_SEND | NA | Pulled to analog GND |
| 8 | Key | – | – |
| **9** | **HPO_L** | **Output** | **Headphone LEFT channel** |
| 10 | BCLK2/GPIO3/PDM_SCL | Input | – |

## CORRECT Wiring for PAM8403 Right Channel

```
Jetson Nano J511 Header
    │
    ├─ Pin 2 (AGND) ──────→ PAM8403 GND
    │                       (Audio ground reference)
    │
    └─ Pin 5 (HPO_R) ──────→ PAM8403 RIN
                            (RIGHT channel audio output)

PAM8403 Amplifier
    │
    ├─ GND ──────────→ (from J511 Pin 2)
    │
    ├─ RIN ──────────→ (from J511 Pin 5)
    │
    └─ R+ and R− ────→ 8Ω Speaker
                       (Right positive/negative)
```

## YOUR CURRENT WIRING (WRONG)

```
Jetson J511 Pin 2 (AGND) → PAM8403 GND ✓ CORRECT
Jetson J511 Pin 9 (HPO_L) → PAM8403 RIN ✗ WRONG!
                           (This is LEFT channel, not RIGHT!)
```

## WHAT YOU NEED TO CHANGE

❌ **Current (WRONG):**
- J511 Pin 9 (HPO_L - LEFT) → PAM8403 RIN

✓ **Should be (CORRECT):**
- J511 Pin 5 (HPO_R - RIGHT) → PAM8403 RIN

## Why This Matters

**PAM8403 has TWO input pins:**
- **LIN** = Left channel input (not used in Phase 1)
- **RIN** = Right channel input (connected to your 8Ω speaker)

**You need to connect:**
- HPO_R (RIGHT output from Jetson) → RIN (Right input to amplifier)
- NOT HPO_L (LEFT output from Jetson)

## How to Fix

You need to **move the wire** from J511 Pin 9 to J511 Pin 5:

1. Unsolder the wire from J511 Pin 9
2. Solder it to J511 Pin 5 (the pin to the left of where it currently is)
3. Verify the solder joint is shiny (not dull/grainy)
4. Run test: `python3 test_audio_fixed.py`
5. You should now hear the beep!

## Quick Reference

```
J511 Header (top view):
    1    3    5    7    9     ← Row A
    2    4    6    8   10     ← Row B
         ↑
         │
    This pin (5) connects to PAM8403 RIN
```

---

**Status:** Wiring corrected - you were using the LEFT channel instead of RIGHT!
**Action:** Move wire from J511 Pin 9 → J511 Pin 5 and resolder
**Expected Result:** Beep will be heard after change!

# LED Board Hardware Verification Guide

**Date:** January 9, 2026  
**Purpose:** Verify LED board wiring before connecting to MCP23017

---

## Step 1: Test Board #1 (Red + Blue LED Board)

### What You Have
- Circular LED PCB with SMD LEDs
- 3-wire connector: Black, Red, Blue
- Built-in resistors (no external resistors needed)

### Test Procedure with Lab Power Supply

**Set power supply to 3.0V, current limit 100mA**

**Test 1 - Red Wire:**
```
Lab Power Supply → LED Board
━━━━━━━━━━━━━━━━━━━━━━━━━━
Negative (GND) ───→ Black wire
Positive (+3V) ────→ Red wire

OBSERVE: Which LEDs light up?
[ ] Red LEDs glow
[ ] Blue LEDs glow
[ ] Other:___________
```

**Test 2 - Blue Wire:**
```
Lab Power Supply → LED Board
━━━━━━━━━━━━━━━━━━━━━━━━━━
Negative (GND) ───→ Black wire
Positive (+3V) ────→ Blue wire

OBSERVE: Which LEDs light up?
[ ] Red LEDs glow
[ ] Blue LEDs glow
[ ] Other:___________
```

**Record Your Results:**
- Red wire controls: __________ LEDs (write "red" or "blue")
- Blue wire controls: __________ LEDs (write "red" or "blue")

**Note:** Wire colors might NOT match LED colors! This is OK - we'll configure it in software.

---

## Step 2: Inspect Board #2 (Yellow + Green LED Board)

### What to Look For

**Green LED location:**
1. Find the green SMD LEDs on Board #2
2. Look for small black components nearby (resistors)
3. Check if resistors are labeled: "220", "221", "470", or similar

**Question to answer:**
- [ ] Green LED has resistor nearby (can desolder both)
- [ ] Green LED has NO resistor (need to add one when soldering to Board #1)

**Photo recommendation:** Take a close-up photo of the green LED area on Board #2 before desoldering.

---

## Step 3: Verify Yellow LED

### Simple 3mm LED Test

```
Lab Power Supply → Yellow LED
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Negative (GND) ───→ 220Ω resistor ───→ Shorter leg (cathode)
Positive (+3V) ───→ Longer leg (anode)

EXPECTED: Yellow LED glows dimly
```

If it doesn't glow, reverse the LED legs (you have polarity backwards).

**Verify:**
- [ ] Yellow LED lights up with 220Ω resistor at 3V
- [ ] Longer leg = voltage side (anode)
- [ ] Shorter leg = ground side (cathode)

---

## Summary - Record Your Findings

**Board #1 Mapping:**
- Red wire → Controls: __________ LEDs
- Blue wire → Controls: __________ LEDs
- Black wire → GND (confirmed)

**Green LED:**
- Has built-in resistor on Board #2: [ ] Yes [ ] No
- Ready to desolder: [ ] Yes [ ] Not yet

**Yellow LED:**
- Tested with 220Ω resistor: [ ] Yes [ ] Not yet
- Polarity confirmed: [ ] Yes [ ] Not yet

---

**Next Steps:**
Once you've verified all hardware, you're ready to:
1. Desolder green LED from Board #2
2. Solder green LED to Board #1
3. Connect everything to MCP23017
4. Run software tests

**Save this checklist** - you'll need the wire mapping for software configuration!


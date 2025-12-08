# Audio Hardware Soldering & Connection Checklist

**Date:** December 7, 2025  
**Status:** Audio not heard - likely solder/connection issue  
**Hardware:** PAM8403 amplifier → 8Ω speaker on J511 RIGHT channel (HPO_R)

---

## Problem Analysis

We've confirmed:
- ✅ ALSA software is working (aplay plays without errors)
- ✅ Mixer is configured (CVB-RT HP at 79%, Speaker at 100%)
- ✅ Audio codec is loaded (CVB-RT Realtek codec active)
- ✅ Device nodes exist (/dev/snd/pcm1d0p)
- ❌ NO SOUND HEARD (despite software working)

**Conclusion:** The problem is **100% HARDWARE** - either solder joints, wiring connections, or component failure.

---

## Soldering Checklist

### 1. J511 Header Connections

**Location:** Small 10-pin header on Jetson near J30

**Check these solder joints visually:**

```
J511 Top View:
┌─────────────────────────────────────┐
│  1    2    3    4    5             │
│ NC  AGND  NC   NC   NC     ← Top row
│
│  6    7    8    9    10            │
│ NC   NC   NC  HPO_L HPO_R   ← Bottom row
└─────────────────────────────────────┘
```

**Visual Inspection for BAD solder:**
- ❌ Dull/grainy appearance (cold joint)
- ❌ Blob of solder not shiny
- ❌ Solder only on one side of pin
- ❌ Wire not fully inserted into pin hole
- ✅ Shiny, smooth, cone-shaped solder joint

**Test each pin:**
- [ ] Pin 2 (AGND): Visual + gentle tug test
- [ ] Pin 9 (HPO_L): Visual + gentle tug test  
- [ ] Pin 10 (HPO_R): Visual + gentle tug test

### 2. PAM8403 Solder Joints

**Pins to check (most critical):**

```
PAM8403 Module (typical DIP package):
        ┌─────────────────┐
   +5V  │ 1           8   │ GND
   GND  │ 2           7   │ RIN
   LIN  │ 3           6   │ GND
   GAIN │ 4           5   │ MUTE
        └─────────────────┘

Speaker outputs (side):
   L+ (left positive)
   L− (left negative)  
   R+ (right positive)
   R− (right negative)
```

**Check these joints:**
- [ ] Pin 1 (+5V): Should be bright/shiny
- [ ] Pin 3 (LIN input): Should be bright/shiny
- [ ] R+ terminal: Solder to speaker wire
- [ ] R− terminal: Solder to speaker wire
- [ ] GND pins: Should be solid

**Visual Red Flags:**
- Solder looking dull/grainy
- Wire stranded copper showing (not covered by solder)
- Solder only partially covering connection point

### 3. Speaker Wire Connections

**Check at PAM8403 output:**
- [ ] R+ wire: Fully inserted, solder shiny and smooth
- [ ] R− wire: Fully inserted, solder shiny and smooth
- [ ] No exposed copper (should be fully soldered)

**Check at speaker terminals:**
- [ ] Red wire (R+): Firmly connected, solder looks good
- [ ] Black wire (R−): Firmly connected, solder looks good
- [ ] No loose strands

---

## Soldering Procedure to Fix Bad Joints

### Equipment Needed:
- Soldering iron (25-40W recommended)
- Solder (lead-free, 60/40 tin/lead, 0.8-1.0mm diameter)
- Solder sucker or desoldering wick (to remove old solder)
- Flux (optional but helps)
- Wire strippers
- Multimeter (to test after)

### Steps for Each Bad Joint:

**For Pin Connections (J511 or PAM8403):**

1. Heat the joint for 2-3 seconds with soldering iron
2. Add a small amount of fresh solder (pea-sized)
3. Remove iron and let cool (wait 5 seconds)
4. Joint should be shiny and smooth (like a cone)

**For PAM8403 Speaker Wire Terminals:**

1. Strip 5-7mm of insulation from wire end
2. Tin the wire (apply solder to bare copper)
3. Tin the PAM8403 terminal
4. Push tinned wire into terminal
5. Heat joint and add small solder amount
6. Let cool - should be shiny

---

## Testing After Soldering

### Visual Test:
- [ ] All joints look shiny and smooth (not dull)
- [ ] No cold solder (grainy appearance)
- [ ] All wires are fully inserted

### Electrical Test (with multimeter):

**Test 1: Continuity Check (beep mode)**
```
1. Set multimeter to Continuity (ohm with speaker symbol)
2. Test J511 Pin 9 → PAM8403 LIN pin
   Expected: BEEP (connected)
3. Test J511 Pin 2 → PAM8403 GND
   Expected: BEEP (connected)
4. Test PAM8403 R+ → Speaker red wire
   Expected: BEEP (connected)
5. Test PAM8403 R− → Speaker black wire
   Expected: BEEP (connected)
```

**Test 2: Voltage Test (during playback)**
```
1. Run: aplay -D hw:1,0 /tmp/long_test.wav
2. While playing, measure with multimeter set to AC voltage:
   - J511 Pin 9 to Pin 2: Should show 0.3-1.0V AC
   - PAM8403 LIN to GND: Should show same voltage
   - PAM8403 R+ to R−: Should show 1-3V AC (amplified)
```

---

## If Still No Sound After Resoldering

**Try these steps:**

1. **Measure with oscilloscope (if available):**
   - Connect probe to J511 Pin 9
   - Run `aplay -D hw:1,0 /tmp/long_test.wav`
   - Should see 1kHz sine wave

2. **Check PAM8403 Power:**
   ```
   With multimeter DC mode:
   - PAM8403 +5V pin: Should read 4.8-5.2V
   - PAM8403 GND pin: Should read 0V
   ```

3. **Check GAIN pin on PAM8403:**
   - If floating (no connection): Default gain is 23dB
   - If PAM8403 has GAIN pin, it should be floating or tied HIGH

4. **Test with different speaker:**
   - Original speaker might be defective
   - Try 8Ω speaker from different source

5. **Check PAM8403 module:**
   - May be counterfeit or damaged
   - Look for burned components
   - Smell for burned solder smell during power up

---

## Successful Audio Setup Checklist

After soldering fix, you should see:

- [ ] aplay command completes without errors
- [ ] Multimeter shows AC voltage on J511 Pin 9 during playback
- [ ] Multimeter shows AC voltage on PAM8403 outputs during playback
- [ ] **HEAR 1kHz BEEP FROM SPEAKER** ✓

---

## Next Steps

1. **Inspect solder joints visually** (strongest candidates for issue)
2. **Reflow the worst-looking joints** (heat + add fresh solder)
3. **Test with continuity meter** (confirm electrical connections)
4. **Rerun audio test:** `python3 audio_beep.py`
5. **Measure voltages** with multimeter while audio plays

---

**Contact:** If still not working after resoldering, we can try:
- Alternative audio output (HDMI)
- Different PAM8403 module
- Debugging codec routing with kernel logs


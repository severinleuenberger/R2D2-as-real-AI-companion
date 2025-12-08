# J511 Pin 5 vs Pin 9 - Which One Do You Need?

## The Problem

You connected your audio wire to **J511 Pin 9** but you should use **J511 Pin 5**.

## What's the Difference?

### J511 Pin 5 (HPO_R) - RIGHT CHANNEL ✓ CORRECT FOR YOU

```
Jetson J511 Pin 5 (HPO_R)
    ↓
    Outputs the RIGHT audio channel from the Jetson codec
    ↓
    PAM8403 RIN (Right Input)
    ↓
    PAM8403 R+ and R− outputs
    ↓
    Your 8Ω speaker
    ↓
    YOU HEAR THE BEEP ✓
```

**This is what you SHOULD be using.**

---

### J511 Pin 9 (HPO_L) - LEFT CHANNEL ✗ WRONG FOR YOU

```
Jetson J511 Pin 9 (HPO_L)
    ↓
    Outputs the LEFT audio channel from the Jetson codec
    ↓
    PAM8403 RIN (Right Input) ← WRONG!
    ↓
    PAM8403 R+ and R− outputs
    ↓
    Your 8Ω speaker (plays LEFT channel audio)
    ↓
    WRONG AUDIO CHANNEL = NO BEEP ✗
```

**This is what you currently have (WRONG).**

---

## Why This Matters

**The PAM8403 has TWO separate inputs:**

```
PAM8403 Amplifier
│
├─ LIN (Left Input)  ← Left channel from Jetson
│   └─ L+ / L− outputs (left speaker)
│
└─ RIN (Right Input) ← Right channel from Jetson
    └─ R+ / R− outputs (your 8Ω speaker)
```

You have your speaker connected to the **RIGHT outputs (R+/R−)**, so you need to send the **RIGHT audio channel (Pin 5)** to the **RIGHT input (RIN)**.

Currently, you're sending the **LEFT channel (Pin 9)** to the **RIGHT input (RIN)**, which causes a channel mismatch.

---

## Visual Comparison

### Current Setup (WRONG)

```
Jetson Codec Output:
    LEFT channel (Pin 9) ──→ PAM8403 RIN (expects RIGHT channel) ✗
    RIGHT channel (Pin 5) ──→ [unused]

Result: No audio on right speaker ✗
```

### Correct Setup (WHAT YOU NEED)

```
Jetson Codec Output:
    LEFT channel (Pin 9) ──→ [unused in Phase 1]
    RIGHT channel (Pin 5) ──→ PAM8403 RIN ✓

Result: Audio plays on right speaker ✓
```

---

## The Fix in 3 Steps

**Step 1:** Unsolder wire from J511 Pin 9
- Heat the solder joint gently
- Pull wire away

**Step 2:** Solder wire to J511 Pin 5
- Position wire on Pin 5 (one pin to the left)
- Heat and add fresh solder
- Should be shiny when cool

**Step 3:** Test
```bash
python3 test_audio_fixed.py
# You should hear the beep now!
```

---

## Key Takeaway

| Aspect | Pin 5 | Pin 9 |
|--------|-------|-------|
| **Name** | HPO_R | HPO_L |
| **Meaning** | Headphone RIGHT | Headphone LEFT |
| **Channel** | RIGHT | LEFT |
| **For your setup** | ✓ CORRECT | ✗ WRONG |
| **Your speaker** | Should receive this | Currently receiving this |

You need Pin 5 because:
1. You want the RIGHT channel
2. Your speaker is on the RIGHT outputs (R+/R−)
3. They need to match!

---

## The Confusion

It's easy to confuse because:
- Both pins are "HPO" (headphone output)
- The naming is HPO_L and HPO_R (not super obvious)
- The pinout isn't symmetrical
- Row A: 1, 3, 5, 7, 9 (odd numbers)
- Row B: 2, 4, 6, 8, 10 (even numbers)

But it's a simple one-pin move to the left!

---

## After You Fix It

Once you move the wire to Pin 5:

```bash
# Verify with continuity test
# J511 Pin 5 should beep when tested to PAM8403 RIN

# Run audio test
python3 test_audio_fixed.py

# Expected: 1kHz beep from speaker ✓
```

The audio system will work!

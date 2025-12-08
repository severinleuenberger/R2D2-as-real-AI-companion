#!/bin/bash
# QUICK FIX - J511 Pin Correction for R2D2 Audio

cat << 'EOF'
╔════════════════════════════════════════════════════════════════════════════╗
║                    QUICK FIX - J511 PIN CORRECTION                        ║
╚════════════════════════════════════════════════════════════════════════════╝

⚠️  ISSUE FOUND
═══════════════════════════════════════════════════════════════════════════════

Your audio wire is connected to the WRONG J511 pin!

CURRENT (WRONG):
  Jetson J511 Pin 9 (HPO_L - LEFT channel) → PAM8403 RIN

SHOULD BE (CORRECT):
  Jetson J511 Pin 5 (HPO_R - RIGHT channel) → PAM8403 RIN

This is why you didn't hear the beep!


📋 J511 HEADER PINOUT
═══════════════════════════════════════════════════════════════════════════════

From official Jetson Nano documentation:

        1    3    5    7    9     ← Row A
        2    4    6    8   10     ← Row B

Key pins:
  Pin 2  = AGND (Audio Ground) ✓ You have this correct!
  Pin 5  = HPO_R (RIGHT audio output) ← MOVE YOUR WIRE HERE
  Pin 9  = HPO_L (LEFT audio output) ← Currently here (WRONG)


🔧 HOW TO FIX (Simple - just move one wire!)
═══════════════════════════════════════════════════════════════════════════════

Step 1: Unsolder the wire from J511 Pin 9
  • Use soldering iron or solder sucker
  • Gently heat the joint until solder melts
  • Pull wire out carefully
  • Let cool

Step 2: Solder the wire to J511 Pin 5
  • Position wire on Pin 5 (one pin to the left of Pin 9)
  • Heat for 2-3 seconds
  • Add small amount of solder (pea-sized)
  • Let cool - should be shiny (not dull)

Step 3: Test
  cd /home/severin/dev/r2d2
  python3 test_audio_fixed.py
  
  You should NOW hear the 1kHz beep!


⚡ KEY DIFFERENCE
═══════════════════════════════════════════════════════════════════════════════

J511 Pin 5 (HPO_R):
  HPO = Headphone output
  R = RIGHT channel
  This connects to your 8Ω speaker via PAM8403

J511 Pin 9 (HPO_L):
  HPO = Headphone output
  L = LEFT channel
  This would connect to left speaker (not used in Phase 1)

You need the RIGHT channel for your RIGHT speaker setup!


📊 VISUAL PIN LAYOUT
═══════════════════════════════════════════════════════════════════════════════

Looking at J511 header from above (component side):

                Pin 1 has a small triangle/square marker
                         │
                         ▼
        ┌──────────────────────────────┐
        │  ▲                            │
Row A:  │  1    3    5    7    9        │  ← Audio signals
        │     ↓ MOVE WIRE HERE         │
Row B:  │  2    4    6    8    10       │
        │       ▲ AGND (correct)       │
        │                               │
        └──────────────────────────────┘

Current wire:  J511 Pin 9 → PAM8403 RIN (WRONG - left channel)
Fixed wire:    J511 Pin 5 → PAM8403 RIN (CORRECT - right channel)


✓ VERIFICATION
═══════════════════════════════════════════════════════════════════════════════

After moving the wire:

1. Check visually:
   □ Wire is soldered to J511 Pin 5 (not Pin 9)
   □ Solder joint is shiny (not dull)
   □ Wire is fully inserted
   □ No loose strands

2. Test connectivity with multimeter:
   □ Set to Continuity mode
   □ Test J511 Pin 5 → PAM8403 RIN
   □ Should BEEP

3. Run audio test:
   python3 test_audio_fixed.py
   You should HEAR the beep!


💡 NOTES
═══════════════════════════════════════════════════════════════════════════════

• This is the ONLY issue with your wiring
• The ground connection (Pin 2) is already correct
• The PAM8403 wiring to speaker is correct
• This is just about which audio channel you use

• Pin 5 is to the LEFT of Pin 9 (one position)
• Easy to mix up - common mistake!


═══════════════════════════════════════════════════════════════════════════════
Action: Move wire from J511 Pin 9 to J511 Pin 5
Time: ~5 minutes
Result: You will hear the beep!
═══════════════════════════════════════════════════════════════════════════════

Need help? See CORRECT_J511_PINOUT.md for official pinout documentation.
EOF

# MCP23017 LED System - Wiring Diagrams

**Visual reference for hardware connections**

---

## Complete System Wiring

```
═══════════════════════════════════════════════════════════════════════════
                        COMPLETE SYSTEM DIAGRAM
═══════════════════════════════════════════════════════════════════════════

JETSON AGX ORIN 40-PIN HEADER                    MCP23017 BOARD
(Looking at board from above)                    (Waveshare Board)
┌───────────────────────────┐                    ┌─────────────────┐
│                           │                    │                 │
│  Pin 1 [3.3V]●  ●[5.0V]   │ Red wire ────────→│ VCC header      │
│  Pin 3 [SDA] ●  ●[5.0V]   │ Yellow wire ─────→│ SDA header      │
│  Pin 5 [SCL] ●  ●[GND]    │ Blue wire ───────→│ SCL header      │
│           Pin 6 ─────────────────────────────→│ GND header      │
│                           │ Black wire         │                 │
│                           │                    │   (Bottom       │
│  ... (other pins)         │                    │    headers)     │
│                           │                    │                 │
│                           │                    ├─────────────────┤
│                           │                    │ LED Outputs:    │
│                           │                    │                 │
│                           │  Board #1 wiring: │ PA0 ◄───┐       │
│                           │  Black ──────────→│ GND ◄───┼─────┐ │
│                           │  Red/Blue ───────→│ PA1 ◄───┘     │ │
│                           │  Green ──────────→│ PA2 ◄─────┐   │ │
│                           │                    │           │   │ │
│                           │  Yellow LED:       │ PA3 ◄─────┼───┼─┤
│                           │  Long leg ───────→│           │   │ │
│                           │  Short leg+220Ω ─→│ GND ◄─────┴───┘ │
│                           │                    │   (Common GND)  │
└───────────────────────────┘                    └─────────────────┘

Legend:
  ● = Filled pin (Pin 1 marker)
  ◄ = Connection point on MCP23017
  → = Wire direction
```

---

## I2C Bus Connection Detail

```
Jetson Pin Layout (Top-down view, Pin 1 at top-left with triangle marker):

      Column 1              Column 2
      (Odd pins)            (Even pins)
      ┌────────┐            ┌────────┐
   1  │ 3.3V ● │            │ ● 5.0V │  2
   3  │ SDA  ● │────────┐   │ ● 5.0V │  4
   5  │ SCL  ● │────┐   │   │ ● GND  │  6  ────┐
   7  │      ● │    │   │   │ ●      │  8      │
   9  │ GND  ● │    │   │   │ ●      │  10     │
      └────────┘    │   │   └────────┘         │
                    │   │                       │
      To MCP23017:  │   │                       │
                    │   │                       │
                  SCL  SDA                     GND
                    │   │                       │
                    ▼   ▼                       ▼
              ┌─────────────────────────────────┐
              │  MCP23017 Bottom Headers        │
              │  [VCC] [GND] [SDA] [SCL]        │
              └─────────────────────────────────┘
                 ▲     ▲
                 │     │
                 │     └──────────────────────────┘
                 │            (From Jetson Pin 6)
                 │
                 └─ From Jetson Pin 1 (3.3V)
```

---

## LED Board #1 Wiring (Pre-Assembled Board)

```
LED Board #1 (Circular PCB)
┌───────────────────────────────┐
│   Multiple SMD LEDs           │
│   (Red and Blue)              │
│                               │
│   Built-in resistors ✅       │
│                               │
│   3-Wire Connector:           │
│   ┌─────────────────┐         │
│   │ Black (GND)     │─────────┼──→ To MCP23017 GND header
│   │ Red (Control)   │─────────┼──→ To MCP23017 PA0 or PA1
│   │ Blue (Control)  │─────────┼──→ To MCP23017 PA1 or PA0
│   └─────────────────┘         │
│                               │
│   After adding green LED:     │
│   ┌─────────────────┐         │
│   │ Green (Control) │─────────┼──→ To MCP23017 PA2
│   └─────────────────┘         │
└───────────────────────────────┘

IMPORTANT: Test with power supply FIRST!
  - Which wire controls which color?
  - Red wire might control blue LEDs (reversed)!
```

---

## Yellow LED Wiring Detail

```
Yellow 3mm LED (Standard 2-leg LED)
┌─────────────────────────────────────────────┐
│                                             │
│     Longer leg (Anode/Voltage side)         │
│           │                                 │
│           │                                 │
│        ───┴───                              │
│       │       │                             │
│       │  LED  │  ← Yellow plastic lens      │
│       │       │                             │
│        ───┬───                              │
│           │                                 │
│           │                                 │
│     Shorter leg (Cathode/Ground side)       │
│           │                                 │
│           │                                 │
│       ┌───┴───┐                             │
│       │ 220Ω  │ ← Current-limiting resistor │
│       │       │    (REQUIRED!)              │
│       └───┬───┘                             │
│           │                                 │
└───────────┼─────────────────────────────────┘
            │
            ▼
        To GND


Connections to MCP23017:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LED longer leg ──────────────→ PA3 header
LED shorter leg → 220Ω → GND ─→ GND header


Common Ground Option (Recommended):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Board #1 Black wire ───┐
Yellow 220Ω resistor ──┼─── Single wire ───→ MCP23017 GND
                       │
(Less wiring, cleaner!)
```

---

## MCP23017 Board Layout Reference

```
Waveshare MCP23017 Board (Top View)
┌─────────────────────────────────────────────────┐
│                                                 │
│  White Connector (Top)                          │
│  [PWR header - not used]                        │
│                                                 │
│  ┌─────────────────────────────────────┐        │
│  │   MCP23017 Chip (IC)                │        │
│  │   (Controls everything)             │        │
│  └─────────────────────────────────────┘        │
│                                                 │
│  Left Side Headers:        Right Side Headers: │
│  PB7 PB6 PB5 PB4           PA7 PA6 PA5 PA4     │
│  PB3 PB2 PB1 PB0           PA3 PA2 PA1 PA0     │
│   │   │   │   │             │   │   │   │      │
│   └───┴───┴───┴─────────────┴───┴───┴───┘      │
│   (Not used)              (Your 4 LEDs)        │
│                                                 │
│  Bottom Headers (I2C Connection):              │
│  [VCC] [GND] [SDA] [SCL]                       │
│    ↑     ↑     ↑     ↑                         │
│    │     │     │     └─ To Jetson Pin 5 (SCL)  │
│    │     │     └─────── To Jetson Pin 3 (SDA)  │
│    │     └───────────── To Jetson Pin 6 (GND)  │
│    └─────────────────── To Jetson Pin 1 (3.3V) │
│                                                 │
│  Address Jumpers (Bottom - Leave OPEN):        │
│  A0 [ ] A1 [ ] A2 [ ]  ← All open = 0x20      │
│                                                 │
└─────────────────────────────────────────────────┘
```

---

## Pin Assignment Summary Table

| LED Color | Function | MCP23017 Pin | Wire Source | Resistor? |
|-----------|----------|--------------|-------------|-----------|
| **Red** | Status: Recognized | PA0 or PA1* | Board #1 | ❌ Built-in |
| **Blue** | Status: No person | PA1 or PA0* | Board #1 | ❌ Built-in |
| **Green** | Status: Unknown | PA2 | Board #1 (add wire) | ❓ Check Board #2 |
| **Yellow** | Gesture flash | PA3 | Simple 3mm LED | ✅ 220Ω external |

\* Test to determine which wire controls which color!

---

## Resistor Calculations (For Reference)

**Why 220Ω?**

```
LED Circuit:
  MCP23017 output: 3.3V
  LED forward voltage: ~2.0V (typical for red/yellow/green)
  Remaining voltage: 3.3V - 2.0V = 1.3V
  
  With 220Ω resistor:
  Current = 1.3V / 220Ω = 5.9mA ✅
  
  This is safe because:
  - MCP23017 max per pin: 25mA (we're at 5.9mA ✅)
  - LED rated current: 15-20mA (we're at 5.9mA ✅)
  - LED brightness: Clearly visible ✅
```

**Alternative values:**
- 150Ω → 8.7mA (brighter)
- 330Ω → 3.9mA (dimmer)
- 470Ω → 2.8mA (very dim)

---

## Common Mistakes to Avoid

❌ **Connecting LED without resistor** → Burns out LED and damages MCP23017!  
❌ **Wrong LED polarity** → LED won't light (but no damage - just reverse it)  
❌ **Swapping SDA/SCL** → I2C won't work (check with i2cdetect)  
❌ **3.3V to GND short** → Damages Jetson! (double-check before power on)  
❌ **Assuming wire colors match LED colors** → Test Board #1 first!  

✅ **Always test with power supply before connecting to MCP23017**  
✅ **Always shutdown Jetson before wiring changes**  
✅ **Always check for shorts with multimeter**  
✅ **Always install resistors on simple LEDs**  

---

## Photos Checklist

**Take photos during installation:**
1. [ ] Board #1 connector close-up (wire colors)
2. [ ] Board #2 green LED area (before desoldering)
3. [ ] Green LED soldered to Board #1 (after)
4. [ ] Yellow LED with 220Ω resistor assembled
5. [ ] Complete MCP23017 wiring (top view)
6. [ ] I2C connection to Jetson pins
7. [ ] All 4 LEDs connected to MCP23017
8. [ ] LEDs lit up (all 4 colors visible)

**Save photos in:** `~/dev/r2d2/_PICTURES/led_installation/`

These will be valuable for:
- Troubleshooting later
- Documentation updates
- Helping others
- Your future reference

---

**Document Status:** Ready for hardware installation  
**Last Updated:** January 9, 2026  
**Reference:** See `270_LED_INSTALLATION.md` for complete guide


# R2D2 Audio Baseline Test Results

> **Note**: This file should be updated after running baseline tests with `test_volume_baseline.py`

## Test Information

**Test Date**: _[Run test_volume_baseline.py to populate]_  
**Hardware**: PAM8403 + 8Ω Speaker (J511 audio)  
**Tester**: _[Your name]_  
**Test Script**: `test_volume_baseline.py --mode interactive`

## Summary

| Audio Source | Min Volume | Default | Max Volume | Optimal Range |
|--------------|------------|---------|------------|---------------|
| Recognition beep | TBD | TBD | TBD | TBD |
| Loss beep | TBD | TBD | TBD | TBD |
| Gesture ack | TBD | TBD | TBD | TBD |
| Session start | TBD | TBD | TBD | TBD |
| Session stop | TBD | TBD | TBD | TBD |
| Speech TTS | TBD | TBD | TBD | TBD |

## Detailed Findings

### Recognition Beeps (Voicy_R2-D2 - 2.mp3)

| Volume | Rating | Notes |
|--------|--------|-------|
| 0.01 | | |
| 0.02 | | |
| 0.05 | | |
| 0.10 | | |
| 0.20 | | |
| 0.30 | | |
| 0.50 | | |
| 0.70 | | |
| 1.00 | | |

### Loss Beeps (Voicy_R2-D2 - 5.mp3)

| Volume | Rating | Notes |
|--------|--------|-------|
| 0.01 | | |
| 0.02 | | |
| 0.05 | | |
| 0.10 | | |
| 0.20 | | |
| 0.30 | | |
| 0.50 | | |
| 0.70 | | |
| 1.00 | | |

### Gesture Acknowledgment (Voicy_R2-D2 - 12.mp3)

| Volume | Rating | Notes |
|--------|--------|-------|
| 0.01 | | |
| 0.02 | | |
| 0.05 | | |
| 0.10 | | |
| 0.20 | | |
| 0.30 | | |
| 0.50 | | |
| 0.70 | | |
| 1.00 | | |

## Rating Scale

| Rating | Description |
|--------|-------------|
| 1 | Too quiet / inaudible |
| 2 | Quiet but audible |
| 3 | Good / acceptable |
| 4 | Loud |
| 5 | Too loud / distortion |

## Recommendations

Based on the baseline tests:

1. **Minimum usable volume**: _TBD_
2. **Maximum safe volume**: _TBD_ (distortion threshold)
3. **Default volume**: _TBD_
4. **Optimal range**: _TBD_

### Volume Curve Recommendation

- [ ] Linear mapping (default)
- [ ] Logarithmic mapping (for perceived loudness)

### Dead Zone Settings

- Low dead zone: _TBD_ (map 0-X% to mute)
- High dead zone: _TBD_ (map 100-X% to max)

## Test JSON File

The detailed JSON results from `test_volume_baseline.py` are stored in:
`test_results/baseline_[DATE].json`

## How to Run Tests

```bash
# Navigate to test directory
cd ros2_ws/src/r2d2_audio/test

# Run interactive baseline test
python3 test_volume_baseline.py --mode interactive --output test_results/baseline_$(date +%Y%m%d).json

# Review results
cat test_results/baseline_*.json | python3 -m json.tool
```

## Verification

After hardware changes or system updates, verify against baseline:

```bash
python3 test_volume_baseline.py --verify test_results/baseline_YYYYMMDD.json
```

## Change History

| Date | Tester | Changes |
|------|--------|---------|
| | | Initial baseline |


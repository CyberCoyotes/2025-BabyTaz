# Vision Tuning Quick Reference

## 🚀 Quick Start (30 seconds)

1. Deploy code to robot
2. Open Shuffleboard
3. Go to "Vision Tests" tab → Click "Model A: Rotation Only"
4. Switch to "Vision Tuning" tab
5. Adjust "A: Rotation kP" slider (try 0.03 → 0.06)
6. Watch robot response change immediately!

## 📊 Key Parameters by Model

### Model A - Rotation Only
| Parameter | Typical Range | Start Value | Purpose |
|-----------|---------------|-------------|---------|
| Rotation kP | 0.03 - 0.08 | 0.05 | Main rotation gain |
| Rotation kD | 0.001 - 0.005 | 0.002 | Damping |
| Max Rotation Speed | 1.0 - 3.0 | 2.0 | Speed limit (rad/s) |
| Min Rotation Speed | 0.1 - 0.3 | 0.2 | Overcome friction |
| Rotation Tolerance | 1.0 - 3.0 | 1.5 | Success threshold (°) |

### Model B - Rotation + Range
| Parameter | Typical Range | Start Value | Purpose |
|-----------|---------------|-------------|---------|
| Rotation kP | 0.03 - 0.05 | 0.035 | Rotation gain |
| Range kP | 0.8 - 1.5 | 1.0 | Forward/back gain |
| Range kD | 0.03 - 0.1 | 0.05 | Forward damping |
| Target Distance | 0.5 - 2.0 | 0.75 | Desired distance (m) |

### Model C - Perpendicular (Full 3-Axis)
| Parameter | Typical Range | Start Value | Purpose |
|-----------|---------------|-------------|---------|
| Rotation kP | 0.04 - 0.08 | 0.05 | Perpendicular align |
| Range kP | 1.0 - 1.5 | 1.2 | Distance control |
| Lateral kP | 0.03 - 0.06 | 0.04 | Side-to-side align |
| Lateral Deadband | 2.0 - 5.0 | 3.0 | Ignore small errors |

### Model D - Color Hunt
| Parameter | Typical Range | Start Value | Purpose |
|-----------|---------------|-------------|---------|
| Hunt Rotation kP | 0.02 - 0.04 | 0.03 | Search rotation |
| Seek Rotation kP | 0.03 - 0.05 | 0.035 | Track rotation |
| Range kP | 0.6 - 1.0 | 0.8 | Approach speed |
| Min Target Area | 0.05 - 0.2 | 0.1 | Filter noise (%) |

## 🎯 Common Tuning Scenarios

### Robot Oscillates (Shakes Back & Forth)
```
❌ Problem: kP too high
✅ Solution:
   1. Reduce kP by 20-30%
   2. Add small kD (0.002 - 0.005)
```

### Robot Stops Short of Target
```
❌ Problem: Min Speed too low or tolerance too tight
✅ Solution:
   1. Increase Min Rotation Speed by 0.05
   2. OR increase tolerance by 0.5°
```

### Robot Overshoots Target
```
❌ Problem: Too aggressive
✅ Solution:
   1. Reduce Max Speed by 0.2
   2. Add kD for damping
   3. Tighten tolerance
```

### Robot Too Slow to Respond
```
❌ Problem: kP too low
✅ Solution:
   1. Increase kP by 0.01 at a time
   2. Test until responsive
   3. Back off if it starts oscillating
```

## 📝 PID Tuning Steps (3 minutes)

1. **Zero out kI and kD** (set to 0.0)
2. **Increase kP** slowly until robot oscillates
3. **Reduce kP** by 50%
4. **Add kD** (start with kP × 0.1)
5. **Test** - adjust kD until stable
6. **Record values** - write them down!
7. **Update VisionConstants.java** when satisfied

## 💾 Saving Your Work

### Step 1: Record Values
Write down successful values from Shuffleboard:
```
Model A:
  ROTATION_KP = 0.052
  ROTATION_KD = 0.003
  MAX_ROTATION_SPEED = 2.2
  ...
```

### Step 2: Update Code
Open `VisionConstants.java`:
```java
public static final class ModelA {
    public static final double ROTATION_KP = 0.052;  // ← Change here
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.003;  // ← Change here
    // ...
}
```

### Step 3: Redeploy
```bash
./gradlew deploy
```

## 🏆 Competition Checklist

Before competition:
- [ ] Tune all models you'll use
- [ ] Update VisionConstants.java with final values
- [ ] Test in competition lighting conditions
- [ ] Lock values with `TunableNumber.setTuningEnabled(false);`
- [ ] Verify tuning tab is disabled

## 🔧 Shuffleboard Layout

```
┌──────────────────────────────────────────────────┐
│ Vision Tests Tab (Run Tests)                     │
│  [Model A]  [Model B]  [Model C]  [Model D]     │
│           [STOP ALL]                             │
│  Active: Model A    Status: ALIGNING             │
│  TX: -2.3°    Distance: 0.85m    ✓ Aligned      │
│  See 'Vision Tuning' tab for live PID adjustment │
└──────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────┐
│ Vision Tuning Tab (Adjust Parameters)            │
│  ┌────────┬────────┬────────┬────────┬─────────┐│
│  │  MAIN  │ MODEL A│ MODEL B│ MODEL C│ MODEL D ││
│  │Forward │  Rot   │  Rot   │  Rot   │  Hunt   ││
│  │Lateral │  kP    │  kP    │  kP    │  Rot kP ││
│  │Rotation│  kI    │  kI    │  kI    │  ...    ││
│  │  ...   │  ...   │  ...   │  ...   │  ...    ││
│  └────────┴────────┴────────┴────────┴─────────┘│
└──────────────────────────────────────────────────┘
```

## ⚡ Pro Tips

1. **Start Simple**: Tune Model A first (rotation only)
2. **One at a Time**: Change one parameter, test, then next
3. **Small Steps**: Adjust by 0.01 for kP, 0.001 for kD
4. **Test Range**: Try at 0.5m, 1.0m, 1.5m distances
5. **Lighting**: Retune if field lighting differs from practice
6. **Document**: Take notes! You'll forget the good values
7. **Backup**: Screenshot good configurations
8. **Competition**: Lock values 30 minutes before matches

## 🐛 Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Values don't change | Tuning disabled | Check `TunableNumber.setTuningEnabled(true)` |
| Tab missing | Not instantiated | Add `new VisionTuningDashboard()` in RobotContainer |
| Sliders don't move | Shuffleboard lag | Restart Shuffleboard |
| Robot ignores changes | Command not running | Check Vision Tests tab shows "RUNNING" |
| Values reset on restart | Not saved to code | Update VisionConstants.java |

## 📚 More Information

- **Full Guide**: `vision-tuning-guide.md`
- **Implementation**: `VISION_TUNING_IMPLEMENTATION_SUMMARY.md`
- **PID Theory**: Search "PID tuning for FRC" on Chief Delphi

---

**Last Updated**: 2026-01-04
**Branch**: vision-testing-2026-01-03

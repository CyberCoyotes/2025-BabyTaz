# Vision Tuning Guide

## Quick Start

The Limelight vision subsystem now includes a dedicated **Vision Tuning** tab in Shuffleboard for live PID tuning without redeploying code.

## How to Use

### 1. Start Robot Code
Deploy and run the robot code. The Vision Tuning tab will automatically appear in Shuffleboard.

### 2. Select a Test Model
Go to the **Vision Tests** tab and click one of the model buttons:
- **Model A**: Rotation Only (align to AprilTag horizontally)
- **Model B**: Rotation + Range (align and maintain distance)
- **Model C**: Perpendicular (full 3-axis alignment)
- **Model D**: Color Hunt (find and align to colored objects)

### 3. Tune Parameters Live
Switch to the **Vision Tuning** tab while the test is running. All parameters are organized by model:

```
┌─────────────────────────────────────────────────────────────────────┐
│ Vision Tuning                                                       │
├─────────────────┬──────────────────┬──────────────────┬────────────┤
│ MAIN COMMAND    │ MODEL A          │ MODEL B          │ MODEL C    │
│ Forward kP      │ A: Rotation kP   │ B: Rotation kP   │ C: Rotation kP │
│ Forward kI      │ A: Rotation kI   │ B: Rotation kI   │ C: Rotation kI │
│ ...             │ ...              │ ...              │ ...        │
└─────────────────┴──────────────────┴──────────────────┴────────────┘
```

### 4. Adjust Values
- **PID Gains**: Use sliders to adjust kP, kI, kD values
- **Speed Limits**: Control maximum rotation/translation speeds
- **Tolerances**: Set acceptable error margins

Changes take effect **immediately** on the next command execution cycle.

## Tuning Tips

### PID Tuning Basics

1. **Start with kP only** (kI = 0, kD = 0)
   - Increase kP until system oscillates
   - Reduce kP by ~50%

2. **Add kD for stability**
   - Add small kD (typically 5-10% of kP)
   - Helps dampen oscillations

3. **Add kI if needed** (usually not required)
   - Only if steady-state error persists
   - Keep very small to avoid instability

### Model-Specific Tips

#### Model A (Rotation Only)
- **kP**: 0.03 - 0.08 (start low)
- **Min Speed**: Increase if robot stalls near target
- **Tolerance**: 1.5° is typical for AprilTags

#### Model B (Rotation + Range)
- **Rotation kP**: Similar to Model A
- **Range kP**: 0.8 - 1.5 (depends on target distance)
- Balance rotation and forward movement

#### Model C (Perpendicular)
- Most complex - tune one axis at a time
- **Rotation**: Align perpendicular to tag face
- **Range**: Control distance to tag
- **Lateral**: Center robot on tag

#### Model D (Color Hunt)
- **Hunt kP**: Lower than seek (slower search)
- **Seek kP**: Higher for responsive tracking
- **Min Target Area**: Filter noise from small detections

## Common Issues

### Robot Oscillates
- **Reduce kP** by 20-30%
- **Add small kD** (0.002 - 0.01)

### Robot Stalls Near Target
- **Increase Min Rotation Speed** (Model A)
- **Check tolerance** - may be too tight

### Robot Overshoots
- **Reduce Max Speed**
- **Add kD** for damping
- **Increase tolerance** slightly

### Slow Response
- **Increase kP**
- **Increase Max Speed**
- Watch for oscillations

## Saving Tuned Values

Once you find good values:

1. **Record values** from Shuffleboard
2. **Update** `VisionConstants.java` with new defaults
3. **Redeploy** code to make permanent

Example:
```java
// In VisionConstants.java - Model A
public static final double ROTATION_KP = 0.05;  // ← Update this
public static final double ROTATION_KI = 0.0;
public static final double ROTATION_KD = 0.002;
```

## Competition Mode

To lock values for competition (prevent accidental changes):

```java
// In RobotContainer or Robot.java
TunableNumber.setTuningEnabled(false);
```

All TunableNumbers will return their default values and ignore Shuffleboard inputs.

## Architecture

### Files Modified/Created

1. **TunableNumber.java** - Enhanced with Shuffleboard support
2. **TunableVisionConstants.java** - Already had all model constants
3. **VisionTuningDashboard.java** - NEW - Organizes display
4. **VisionTestDashboard.java** - Updated with tuning tab reference
5. **ModelA.java** - Already using TunableVisionConstants

### How It Works

```
ModelA.execute()
    ↓
TunableVisionConstants.ModelA.ROTATION_KP.get()
    ↓
SmartDashboard.getNumber("Vision/ModelA/Rotation_kP", default)
    ↓
Returns current value from Shuffleboard slider
```

Changes propagate immediately because:
1. Commands call `.get()` every execution cycle
2. `.hasChanged()` triggers PID controller updates
3. No code redeployment needed

## Technical Details

### TunableNumber Features
- Two constructors: SmartDashboard-only and Shuffleboard-integrated
- Change detection with epsilon comparison (1e-9)
- Competition mode toggle
- Automatic range limits based on parameter type

### Performance
- Minimal overhead: ~0.1ms per value read
- No network bottleneck: uses NetworkTables efficiently
- Values cached between reads

## Troubleshooting

### Values Don't Change
- Check TunableNumber.setTuningEnabled(true)
- Verify command is running (check Vision Tests tab)
- Confirm slider is moving in Shuffleboard

### Shuffleboard Laggy
- Too many NetworkTables updates
- Close unused tabs
- Reduce logging frequency if using AdvantageKit

### Values Reset
- TunableNumbers initialize to defaults on robot restart
- Save good values to VisionConstants.java
- Use reset button to return to defaults during testing

## Next Steps

1. Tune Model A first (simplest)
2. Move to Model B once comfortable
3. Record all successful values
4. Update VisionConstants.java before competition
5. Enable competition mode for matches

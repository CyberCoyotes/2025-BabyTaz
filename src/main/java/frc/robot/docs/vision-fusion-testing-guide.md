# Vision Fusion Testing & Analysis Guide

## Overview

This guide explains how to run drift tests, validate vision fusion, and analyze the results in AdvantageScope. The goal is to quantify how much vision correction improves your autonomous positioning.

---

## Quick Start

### Step 1: Add Commands to RobotContainer

```java
// In RobotContainer.java

// Create the pose fusion subsystem
private final PoseFusionSubsystem poseFusion = 
    new PoseFusionSubsystem(drivetrain, "limelight");

// Add test commands to SmartDashboard
public RobotContainer() {
    // ... existing code ...
    
    // Testing commands
    SmartDashboard.putData("Run Drift Test", 
        new DriftTestCommand(drivetrain));
    
    SmartDashboard.putData("Run Vision Validation", 
        new VisionValidationCommand(drivetrain, poseFusion));
    
    SmartDashboard.putData("Toggle Fusion", 
        Commands.runOnce(() -> poseFusion.setFusionEnabled(!poseFusion.isFusionEnabled())));
    
    SmartDashboard.putData("Reset Fusion Stats",
        Commands.runOnce(() -> poseFusion.resetStatistics()));
}

// Make sure poseFusion runs every cycle
@Override
public void robotPeriodic() {
    // poseFusion is a subsystem, so it runs automatically
}
```

### Step 2: Run the Tests

1. **Place robot in test area:**
   - Needs 2m x 2m clear space
   - Should be able to see AprilTags during the test
   - Mark starting position with tape

2. **Run Drift Test first (odometry baseline):**
   - Click "Run Drift Test" in SmartDashboard
   - Robot drives a square pattern
   - Watch console for results

3. **Run Vision Validation (comparison test):**
   - Click "Run Vision Validation"
   - Robot runs the pattern twice (fusion off, then fusion on)
   - Results printed to console and logged

### Step 3: Download Logs

```bash
# From your laptop, connected to robot:
scp admin@10.XX.XX.2:/home/lvuser/logs/*.wpilog ~/Desktop/FRC-Logs/
```

Or use AdvantageScope's built-in download feature.

---

## What to Look For in AdvantageScope

### Key Logged Values

Open your `.wpilog` file in AdvantageScope and examine these values:

#### Drift Test Results

| Log Path | What It Tells You |
|----------|-------------------|
| `DriftTest/Final/TranslationDriftMagnitude` | Total position error after returning to start (meters) |
| `DriftTest/Final/DriftPerMeter` | Normalized drift - **this is your key metric!** |
| `DriftTest/Final/RotationDriftDeg` | Heading error in degrees |
| `DriftTest/Live/TranslationErrorMagnitude` | Error over time during test |

**Interpreting Drift Per Meter:**
- `< 0.01` (< 1%): Excellent odometry
- `0.01 - 0.02` (1-2%): Good, vision will help
- `0.02 - 0.05` (2-5%): Moderate drift, vision strongly recommended
- `> 0.05` (> 5%): High drift - check wheel calibration

#### Vision Validation Results

| Log Path | What It Tells You |
|----------|-------------------|
| `VisionValidation/Results/OdometryOnly/FinalDrift` | Error without vision |
| `VisionValidation/Results/WithFusion/FinalDrift` | Error with vision |
| `VisionValidation/Results/ImprovementPercent` | How much better fusion is |

**Expected Improvement:** 30-70% reduction in drift with working fusion

#### Pose Fusion Status

| Log Path | What It Tells You |
|----------|-------------------|
| `PoseFusion/AcceptedCount` | How many vision measurements were used |
| `PoseFusion/RejectedCount` | How many were thrown out |
| `PoseFusion/AcceptRate` | Ratio of accepted to total |
| `PoseFusion/LastRejectReason` | Why measurements are being rejected |
| `PoseFusion/CumulativeCorrection` | Total meters of correction applied |

**Healthy Fusion Signs:**
- Accept rate > 70%
- Corrections happening during tag visibility
- No constant reject reason

---

## Visualizing in AdvantageScope

### Recommended Dashboard Setup

1. **Create a new tab called "Vision Fusion"**

2. **Add a Field2d view:**
   - Drag `DriftTest/CurrentPose` to see robot position
   - Also add `DriftTest/Final/StartPose` and `DriftTest/Final/EndPose`

3. **Add line graphs for:**
   - `DriftTest/Live/TranslationErrorMagnitude` - See drift accumulate
   - `PoseFusion/CumulativeCorrection` - See vision corrections happen

4. **Add a table showing:**
   - `PoseFusion/AcceptedCount`
   - `PoseFusion/RejectedCount`
   - `PoseFusion/LastRejectReason`

### Comparing Runs

To compare odometry-only vs fusion:

1. Open two log files side-by-side (if you ran tests separately)
2. Or look at `VisionValidation/OdometryOnly/*` vs `VisionValidation/WithFusion/*`
3. Overlay the `TranslationErrorMagnitude` graphs to see difference

---

## Troubleshooting

### Problem: Vision isn't helping (or making things worse)

**Check these in the logs:**

1. **Is fusion accepting measurements?**
   - Look at `PoseFusion/AcceptRate`
   - If < 50%, too many rejections

2. **Why are measurements rejected?**
   - Check `PoseFusion/LastRejectReason`
   
   | Reject Reason | Fix |
   |---------------|-----|
   | `NO_TAGS` | Robot can't see AprilTags - check field of view |
   | `SPINNING_TOO_FAST` | Reduce test speed or increase threshold |
   | `TAG_TOO_SMALL` | Robot too far from tags - reduce MIN_TAG_AREA |
   | `POSE_JUMP` | Calibration issue or bad mounting |

3. **Are standard deviations tuned correctly?**
   - If fusion overcorrects, increase std devs (less trust)
   - If fusion barely helps, decrease std devs (more trust)

### Problem: High baseline drift (even without vision issues)

**Things to check:**
- Wheel diameter calibration
- Tread wear (measure actual wheel circumference)
- Encoder resolution settings
- Module alignment / CANcoder offsets

### Problem: Inconsistent results between runs

**Likely causes:**
- Different starting positions (tag visibility varies)
- Battery voltage affecting motor performance
- Floor surface differences

**Solution:** Run multiple tests and average results

---

## Sample Test Report

After running tests, you should be able to fill in a report like this:

```
=== VISION FUSION TEST REPORT ===
Date: ___________
Robot: Baby Taz

BASELINE DRIFT TEST:
- Square size: 2.0m (8m total path)
- Number of squares: 2
- Final position error: _____ meters
- Drift per meter: _____%
- Rotation drift: _____ degrees

VISION VALIDATION:
- Odometry-only drift: _____ meters
- Fusion-enabled drift: _____ meters  
- Improvement: _____%
- Vision measurements accepted: _____
- Vision measurements rejected: _____
- Rejection rate: _____%
- Most common reject reason: _________

CONCLUSIONS:
- Odometry quality: [ ] Excellent  [ ] Good  [ ] Needs work
- Fusion improvement: [ ] Significant  [ ] Moderate  [ ] None
- Recommended actions: ________________
```

---

## Advanced: Tuning Standard Deviations

If you want to fine-tune the fusion behavior:

### Current Settings (in PoseFusionSubsystem)

```java
// Multi-tag (higher trust)
MULTI_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.5);

// Single-tag (lower trust)
SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
```

### Tuning Process

1. **Start conservative** (higher std devs = less trust)
2. **Run validation test**
3. **If fusion helps but could help more:** Lower std devs by 0.1
4. **If fusion causes oscillation:** Raise std devs by 0.1
5. **Repeat until optimal**

### What "Optimal" Looks Like

- Position settles quickly after vision update
- No visible oscillation or "hunting"
- Drift reduction of 50%+ in validation test
- Robot follows paths more accurately

---

## Questions for Discussion with Students

1. **Why do we use different std devs for single vs multi-tag?**
   - Multi-tag gives more geometric constraints
   - Single tag can have ambiguous solutions

2. **Why do we never trust single-tag heading?**
   - Your gyro is better than vision for heading
   - Single-tag heading can flip 180°

3. **What happens if we trust vision too much?**
   - Position jumps when tags appear/disappear
   - Can cause jerky autonomous movement

4. **What happens if we trust vision too little?**
   - Odometry drift isn't corrected
   - End up in wrong position at end of auto

---

## File Locations

After adding these files to your project:

```
src/main/java/frc/robot/
├── commands/
│   └── testing/
│       ├── DriftTestCommand.java
│       └── VisionValidationCommand.java
├── subsystems/
│   └── PoseFusionSubsystem.java
└── documentation/
    └── vision-fusion-testing-guide.md  (this file)
```

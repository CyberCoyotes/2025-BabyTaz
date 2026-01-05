package frc.robot.subsystems.vision;

import frc.robot.util.TunableNumber;

/**
 * TunableVisionConstants - Live-tunable PID and vision parameters.
 *
 * This class wraps all tunable vision alignment parameters using TunableNumber,
 * allowing real-time adjustment via SmartDashboard/Shuffleboard without redeploying.
 *
 * Usage:
 * - View/edit values in SmartDashboard or Shuffleboard
 * - Values are organized by model (A, B, C, D) and main AlignToAprilTagCommand
 * - Changes take effect immediately on next command execution
 * - Disable tuning for competition: TunableNumber.setTuningEnabled(false)
 */
public class TunableVisionConstants {

    // ============================================================================
    // MAIN ALIGNMENT COMMAND (AlignToAprilTagCommand)
    // ============================================================================

    public static class Main {
        // Forward/Backward PID
        public static final TunableNumber FORWARD_KP =
            new TunableNumber("Vision/Main/Forward_kP", 1.0);
        public static final TunableNumber FORWARD_KI =
            new TunableNumber("Vision/Main/Forward_kI", 0.0);
        public static final TunableNumber FORWARD_KD =
            new TunableNumber("Vision/Main/Forward_kD", 0.0);

        // Lateral/Strafe PID
        public static final TunableNumber LATERAL_KP =
            new TunableNumber("Vision/Main/Lateral_kP", 0.3);
        public static final TunableNumber LATERAL_KI =
            new TunableNumber("Vision/Main/Lateral_kI", 0.0);
        public static final TunableNumber LATERAL_KD =
            new TunableNumber("Vision/Main/Lateral_kD", 0.0);

        // Rotation PID
        public static final TunableNumber ROTATION_KP =
            new TunableNumber("Vision/Main/Rotation_kP", 0.08);
        public static final TunableNumber ROTATION_KI =
            new TunableNumber("Vision/Main/Rotation_kI", 0.0);
        public static final TunableNumber ROTATION_KD =
            new TunableNumber("Vision/Main/Rotation_kD", 0.0);

        // Speed limits
        public static final TunableNumber MAX_FORWARD_SPEED =
            new TunableNumber("Vision/Main/MaxForwardSpeed", 0.625);
        public static final TunableNumber MAX_LATERAL_SPEED =
            new TunableNumber("Vision/Main/MaxLateralSpeed", 0.625);
        public static final TunableNumber MAX_ROTATION_SPEED =
            new TunableNumber("Vision/Main/MaxRotationSpeed", 0.9375);

        // Tolerances
        public static final TunableNumber FORWARD_TOLERANCE =
            new TunableNumber("Vision/Main/ForwardTolerance_m", 0.10);
        public static final TunableNumber LATERAL_TOLERANCE =
            new TunableNumber("Vision/Main/LateralTolerance_m", 0.05);
        public static final TunableNumber ROTATION_TOLERANCE =
            new TunableNumber("Vision/Main/RotationTolerance_deg", 2.0);

        // Target distance
        public static final TunableNumber TARGET_DISTANCE =
            new TunableNumber("Vision/Main/TargetDistance_m", 1.0);
    }

    // ============================================================================
    // MODEL A: ROTATION ONLY
    // ============================================================================

    public static class ModelA {
        public static final TunableNumber ROTATION_KP =
            new TunableNumber("Vision/ModelA/Rotation_kP", VisionConstants.ModelA.ROTATION_KP);
        public static final TunableNumber ROTATION_KI =
            new TunableNumber("Vision/ModelA/Rotation_kI", VisionConstants.ModelA.ROTATION_KI);
        public static final TunableNumber ROTATION_KD =
            new TunableNumber("Vision/ModelA/Rotation_kD", VisionConstants.ModelA.ROTATION_KD);

        public static final TunableNumber MAX_ROTATION_SPEED =
            new TunableNumber("Vision/ModelA/MaxRotationSpeed", VisionConstants.ModelA.MAX_ROTATION_SPEED_RADPS);
        public static final TunableNumber MIN_ROTATION_SPEED =
            new TunableNumber("Vision/ModelA/MinRotationSpeed", VisionConstants.ModelA.MIN_ROTATION_SPEED_RADPS);

        public static final TunableNumber ROTATION_TOLERANCE =
            new TunableNumber("Vision/ModelA/RotationTolerance_deg", VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES);

        // Convenience getters for commands
        private static double lastKp, lastKi, lastKd, lastTolerance;

        public static double getRotationKp() { return ROTATION_KP.get(); }
        public static double getRotationKi() { return ROTATION_KI.get(); }
        public static double getRotationKd() { return ROTATION_KD.get(); }
        public static double getMaxRotationSpeed() { return MAX_ROTATION_SPEED.get(); }
        public static double getMinRotationSpeed() { return MIN_ROTATION_SPEED.get(); }
        public static double getRotationTolerance() { return ROTATION_TOLERANCE.get(); }

        public static boolean anyPIDHasChanged() {
            return ROTATION_KP.hasChanged() || ROTATION_KI.hasChanged() || ROTATION_KD.hasChanged();
        }

        public static boolean rotationToleranceHasChanged() {
            return ROTATION_TOLERANCE.hasChanged();
        }
    }

    // ============================================================================
    // MODEL B: ROTATION + RANGE
    // ============================================================================

    public static class ModelB {
        // Rotation PID
        public static final TunableNumber ROTATION_KP =
            new TunableNumber("Vision/ModelB/Rotation_kP", VisionConstants.ModelB.ROTATION_KP);
        public static final TunableNumber ROTATION_KI =
            new TunableNumber("Vision/ModelB/Rotation_kI", VisionConstants.ModelB.ROTATION_KI);
        public static final TunableNumber ROTATION_KD =
            new TunableNumber("Vision/ModelB/Rotation_kD", VisionConstants.ModelB.ROTATION_KD);

        // Range PID
        public static final TunableNumber RANGE_KP =
            new TunableNumber("Vision/ModelB/Range_kP", VisionConstants.ModelB.RANGE_KP);
        public static final TunableNumber RANGE_KI =
            new TunableNumber("Vision/ModelB/Range_kI", VisionConstants.ModelB.RANGE_KI);
        public static final TunableNumber RANGE_KD =
            new TunableNumber("Vision/ModelB/Range_kD", VisionConstants.ModelB.RANGE_KD);

        // Speed limits
        public static final TunableNumber MAX_ROTATION_SPEED =
            new TunableNumber("Vision/ModelB/MaxRotationSpeed", VisionConstants.ModelB.MAX_ROTATION_SPEED_RADPS);
        public static final TunableNumber MAX_FORWARD_SPEED =
            new TunableNumber("Vision/ModelB/MaxForwardSpeed", VisionConstants.ModelB.MAX_FORWARD_SPEED_MPS);

        // Tolerances
        public static final TunableNumber ROTATION_TOLERANCE =
            new TunableNumber("Vision/ModelB/RotationTolerance_deg", VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES);
        public static final TunableNumber DISTANCE_TOLERANCE =
            new TunableNumber("Vision/ModelB/DistanceTolerance_m", VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS);

        // Target distance
        public static final TunableNumber TARGET_DISTANCE =
            new TunableNumber("Vision/ModelB/TargetDistance_m", VisionConstants.DEFAULT_TARGET_DISTANCE_METERS);

        // Convenience getters for commands
        public static double getRotationKp() { return ROTATION_KP.get(); }
        public static double getRotationKi() { return ROTATION_KI.get(); }
        public static double getRotationKd() { return ROTATION_KD.get(); }
        public static double getRangeKp() { return RANGE_KP.get(); }
        public static double getRangeKi() { return RANGE_KI.get(); }
        public static double getRangeKd() { return RANGE_KD.get(); }
        public static double getMaxRotationSpeed() { return MAX_ROTATION_SPEED.get(); }
        public static double getMaxForwardSpeed() { return MAX_FORWARD_SPEED.get(); }
        public static double getRotationTolerance() { return ROTATION_TOLERANCE.get(); }
        public static double getDistanceTolerance() { return DISTANCE_TOLERANCE.get(); }
        public static double getTargetDistance() { return TARGET_DISTANCE.get(); }

        public static boolean rotationPIDHasChanged() {
            return ROTATION_KP.hasChanged() || ROTATION_KI.hasChanged() || ROTATION_KD.hasChanged();
        }

        public static boolean rangePIDHasChanged() {
            return RANGE_KP.hasChanged() || RANGE_KI.hasChanged() || RANGE_KD.hasChanged();
        }

        public static boolean rotationToleranceHasChanged() {
            return ROTATION_TOLERANCE.hasChanged();
        }

        public static boolean distanceToleranceHasChanged() {
            return DISTANCE_TOLERANCE.hasChanged();
        }

        public static boolean targetDistanceHasChanged() {
            return TARGET_DISTANCE.hasChanged();
        }
    }

    // ============================================================================
    // MODEL C: PERPENDICULAR ALIGNMENT (3-axis)
    // ============================================================================

    public static class ModelC {
        // Rotation PID
        public static final TunableNumber ROTATION_KP =
            new TunableNumber("Vision/ModelC/Rotation_kP", VisionConstants.ModelC.ROTATION_KP);
        public static final TunableNumber ROTATION_KI =
            new TunableNumber("Vision/ModelC/Rotation_kI", VisionConstants.ModelC.ROTATION_KI);
        public static final TunableNumber ROTATION_KD =
            new TunableNumber("Vision/ModelC/Rotation_kD", VisionConstants.ModelC.ROTATION_KD);

        // Range PID
        public static final TunableNumber RANGE_KP =
            new TunableNumber("Vision/ModelC/Range_kP", VisionConstants.ModelC.RANGE_KP);
        public static final TunableNumber RANGE_KI =
            new TunableNumber("Vision/ModelC/Range_kI", VisionConstants.ModelC.RANGE_KI);
        public static final TunableNumber RANGE_KD =
            new TunableNumber("Vision/ModelC/Range_kD", VisionConstants.ModelC.RANGE_KD);

        // Lateral PID
        public static final TunableNumber LATERAL_KP =
            new TunableNumber("Vision/ModelC/Lateral_kP", VisionConstants.ModelC.LATERAL_KP);
        public static final TunableNumber LATERAL_KI =
            new TunableNumber("Vision/ModelC/Lateral_kI", VisionConstants.ModelC.LATERAL_KI);
        public static final TunableNumber LATERAL_KD =
            new TunableNumber("Vision/ModelC/Lateral_kD", VisionConstants.ModelC.LATERAL_KD);

        // Speed limits
        public static final TunableNumber MAX_ROTATION_SPEED =
            new TunableNumber("Vision/ModelC/MaxRotationSpeed", VisionConstants.ModelC.MAX_ROTATION_SPEED_RADPS);
        public static final TunableNumber MAX_FORWARD_SPEED =
            new TunableNumber("Vision/ModelC/MaxForwardSpeed", VisionConstants.ModelC.MAX_FORWARD_SPEED_MPS);
        public static final TunableNumber MAX_LATERAL_SPEED =
            new TunableNumber("Vision/ModelC/MaxLateralSpeed", VisionConstants.ModelC.MAX_LATERAL_SPEED_MPS);

        // Tolerances
        public static final TunableNumber ROTATION_TOLERANCE =
            new TunableNumber("Vision/ModelC/RotationTolerance_deg", VisionConstants.ModelC.ROTATION_TOLERANCE_DEGREES);
        public static final TunableNumber DISTANCE_TOLERANCE =
            new TunableNumber("Vision/ModelC/DistanceTolerance_m", VisionConstants.ModelC.DISTANCE_TOLERANCE_METERS);
        public static final TunableNumber LATERAL_TOLERANCE =
            new TunableNumber("Vision/ModelC/LateralTolerance_deg", VisionConstants.ModelC.LATERAL_TOLERANCE_DEGREES);

        // Lateral deadband
        public static final TunableNumber LATERAL_DEADBAND =
            new TunableNumber("Vision/ModelC/LateralDeadband_deg", VisionConstants.ModelC.LATERAL_DEADBAND_DEGREES);

        // Target distance
        public static final TunableNumber TARGET_DISTANCE =
            new TunableNumber("Vision/ModelC/TargetDistance_m", VisionConstants.DEFAULT_TARGET_DISTANCE_METERS);
    }

    // ============================================================================
    // MODEL D: COLOR BLOB HUNT
    // ============================================================================

    public static class ModelD {
        // Hunt mode PID
        public static final TunableNumber HUNT_ROTATION_KP =
            new TunableNumber("Vision/ModelD/HuntRotation_kP", VisionConstants.ModelD.HUNT_ROTATION_KP);
        public static final TunableNumber HUNT_ROTATION_KI =
            new TunableNumber("Vision/ModelD/HuntRotation_kI", VisionConstants.ModelD.HUNT_ROTATION_KI);
        public static final TunableNumber HUNT_ROTATION_KD =
            new TunableNumber("Vision/ModelD/HuntRotation_kD", VisionConstants.ModelD.HUNT_ROTATION_KD);

        // Seek mode rotation PID
        public static final TunableNumber SEEK_ROTATION_KP =
            new TunableNumber("Vision/ModelD/SeekRotation_kP", VisionConstants.ModelD.SEEK_ROTATION_KP);
        public static final TunableNumber SEEK_ROTATION_KI =
            new TunableNumber("Vision/ModelD/SeekRotation_kI", VisionConstants.ModelD.SEEK_ROTATION_KI);
        public static final TunableNumber SEEK_ROTATION_KD =
            new TunableNumber("Vision/ModelD/SeekRotation_kD", VisionConstants.ModelD.SEEK_ROTATION_KD);

        // Range PID
        public static final TunableNumber RANGE_KP =
            new TunableNumber("Vision/ModelD/Range_kP", VisionConstants.ModelD.RANGE_KP);
        public static final TunableNumber RANGE_KI =
            new TunableNumber("Vision/ModelD/Range_kI", VisionConstants.ModelD.RANGE_KI);
        public static final TunableNumber RANGE_KD =
            new TunableNumber("Vision/ModelD/Range_kD", VisionConstants.ModelD.RANGE_KD);

        // Speed limits
        public static final TunableNumber HUNT_ROTATION_SPEED =
            new TunableNumber("Vision/ModelD/HuntRotationSpeed", VisionConstants.ModelD.HUNT_ROTATION_SPEED_RADPS);
        public static final TunableNumber MAX_ROTATION_SPEED =
            new TunableNumber("Vision/ModelD/MaxRotationSpeed", VisionConstants.ModelD.MAX_ROTATION_SPEED_RADPS);
        public static final TunableNumber MAX_FORWARD_SPEED =
            new TunableNumber("Vision/ModelD/MaxForwardSpeed", VisionConstants.ModelD.MAX_FORWARD_SPEED_MPS);

        // Tolerances
        public static final TunableNumber ROTATION_TOLERANCE =
            new TunableNumber("Vision/ModelD/RotationTolerance_deg", VisionConstants.ModelD.ROTATION_TOLERANCE_DEGREES);
        public static final TunableNumber DISTANCE_TOLERANCE =
            new TunableNumber("Vision/ModelD/DistanceTolerance_m", VisionConstants.ModelD.DISTANCE_TOLERANCE_METERS);

        // Color blob thresholds
        public static final TunableNumber MIN_TARGET_AREA =
            new TunableNumber("Vision/ModelD/MinTargetArea", VisionConstants.ModelD.MIN_TARGET_AREA);
        public static final TunableNumber TARGET_AREA_FOR_DISTANCE =
            new TunableNumber("Vision/ModelD/TargetAreaForDistance", VisionConstants.ModelD.TARGET_AREA_FOR_DISTANCE);
    }

    // ============================================================================
    // CAMERA CONFIGURATION (less frequently tuned, but available)
    // ============================================================================

    public static class Camera {
        public static final TunableNumber HEIGHT =
            new TunableNumber("Vision/Camera/Height_m", VisionConstants.CAMERA_HEIGHT_METERS);
        public static final TunableNumber ANGLE =
            new TunableNumber("Vision/Camera/Angle_deg", VisionConstants.CAMERA_ANGLE_DEGREES);
        public static final TunableNumber LATERAL_OFFSET =
            new TunableNumber("Vision/Camera/LateralOffset_m", VisionConstants.CAMERA_LATERAL_OFFSET_METERS);
        public static final TunableNumber LONGITUDINAL_OFFSET =
            new TunableNumber("Vision/Camera/LongitudinalOffset_m", VisionConstants.CAMERA_LONGITUDINAL_OFFSET_METERS);
    }

    /**
     * Initialize all tunable constants. Called during robot initialization.
     * Note: With TunableNumber approach, initialization happens automatically
     * when the static fields are first accessed. This method exists for
     * compatibility with commands that expect it.
     */
    public static void initializeAll() {
        // TunableNumbers are initialized automatically when first accessed
        // This method is a no-op but exists for API compatibility
    }

    /**
     * Reset all tunable values to their defaults.
     * Useful for resetting after testing.
     */
    public static void resetAllToDefaults() {
        // Model A
        ModelA.ROTATION_KP.reset();
        ModelA.ROTATION_KI.reset();
        ModelA.ROTATION_KD.reset();
        ModelA.MAX_ROTATION_SPEED.reset();
        ModelA.MIN_ROTATION_SPEED.reset();
        ModelA.ROTATION_TOLERANCE.reset();

        // Model B
        ModelB.ROTATION_KP.reset();
        ModelB.ROTATION_KI.reset();
        ModelB.ROTATION_KD.reset();
        ModelB.RANGE_KP.reset();
        ModelB.RANGE_KI.reset();
        ModelB.RANGE_KD.reset();
        ModelB.MAX_ROTATION_SPEED.reset();
        ModelB.MAX_FORWARD_SPEED.reset();
        ModelB.ROTATION_TOLERANCE.reset();
        ModelB.DISTANCE_TOLERANCE.reset();
        ModelB.TARGET_DISTANCE.reset();

        // Model C
        ModelC.ROTATION_KP.reset();
        ModelC.ROTATION_KI.reset();
        ModelC.ROTATION_KD.reset();
        ModelC.RANGE_KP.reset();
        ModelC.RANGE_KI.reset();
        ModelC.RANGE_KD.reset();
        ModelC.LATERAL_KP.reset();
        ModelC.LATERAL_KI.reset();
        ModelC.LATERAL_KD.reset();
        ModelC.MAX_ROTATION_SPEED.reset();
        ModelC.MAX_FORWARD_SPEED.reset();
        ModelC.MAX_LATERAL_SPEED.reset();
        ModelC.ROTATION_TOLERANCE.reset();
        ModelC.DISTANCE_TOLERANCE.reset();
        ModelC.LATERAL_TOLERANCE.reset();
        ModelC.LATERAL_DEADBAND.reset();
        ModelC.TARGET_DISTANCE.reset();

        // Model D
        ModelD.HUNT_ROTATION_KP.reset();
        ModelD.HUNT_ROTATION_KI.reset();
        ModelD.HUNT_ROTATION_KD.reset();
        ModelD.SEEK_ROTATION_KP.reset();
        ModelD.SEEK_ROTATION_KI.reset();
        ModelD.SEEK_ROTATION_KD.reset();
        ModelD.RANGE_KP.reset();
        ModelD.RANGE_KI.reset();
        ModelD.RANGE_KD.reset();
        ModelD.HUNT_ROTATION_SPEED.reset();
        ModelD.MAX_ROTATION_SPEED.reset();
        ModelD.MAX_FORWARD_SPEED.reset();
        ModelD.ROTATION_TOLERANCE.reset();
        ModelD.DISTANCE_TOLERANCE.reset();
        ModelD.MIN_TARGET_AREA.reset();
        ModelD.TARGET_AREA_FOR_DISTANCE.reset();

        // Main
        Main.FORWARD_KP.reset();
        Main.FORWARD_KI.reset();
        Main.FORWARD_KD.reset();
        Main.LATERAL_KP.reset();
        Main.LATERAL_KI.reset();
        Main.LATERAL_KD.reset();
        Main.ROTATION_KP.reset();
        Main.ROTATION_KI.reset();
        Main.ROTATION_KD.reset();
        Main.MAX_FORWARD_SPEED.reset();
        Main.MAX_LATERAL_SPEED.reset();
        Main.MAX_ROTATION_SPEED.reset();
        Main.FORWARD_TOLERANCE.reset();
        Main.LATERAL_TOLERANCE.reset();
        Main.ROTATION_TOLERANCE.reset();
        Main.TARGET_DISTANCE.reset();

        // Camera
        Camera.HEIGHT.reset();
        Camera.ANGLE.reset();
        Camera.LATERAL_OFFSET.reset();
        Camera.LONGITUDINAL_OFFSET.reset();
    }
}

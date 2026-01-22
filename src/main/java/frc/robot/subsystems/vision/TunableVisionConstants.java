package frc.robot.subsystems.vision;

import frc.robot.util.TunableNumber;

@SuppressWarnings("unused")

/**
 * TunableVisionConstants - Live-tunable PID and vision parameters with Elastic Dashboard integration.
 *
 * This class creates organized NetworkTables entries for each vision alignment model,
 * allowing real-time adjustment of PID gains, speeds, and tolerances without redeploying.
 *
 * Usage:
 * - Call initializeAll() during robot initialization to create all NetworkTables entries
 * - Each model has its own dedicated tab in Elastic: "Vision-Main", "Vision-ModelA", etc.
 * - Values can be adjusted via Elastic Dashboard widgets
 * - Changes take effect immediately on next command execution
 * - Disable tuning for competition: TunableNumber.setTuningEnabled(false)
 */
public class TunableVisionConstants {

    private static boolean initialized = false;

    // ============================================================================
    // MAIN ALIGNMENT COMMAND (AlignToAprilTagCommand)
    // ============================================================================

    public static class Main {
        private static final String TAB_NAME = "Vision-Main";

        // Forward/Backward PID
        public static TunableNumber FORWARD_KP;
        public static TunableNumber FORWARD_KI;
        public static TunableNumber FORWARD_KD;

        // Lateral/Strafe PID
        public static TunableNumber LATERAL_KP;
        public static TunableNumber LATERAL_KI;
        public static TunableNumber LATERAL_KD;

        // Rotation PID
        public static TunableNumber ROTATION_KP;
        public static TunableNumber ROTATION_KI;
        public static TunableNumber ROTATION_KD;

        // Speed limits
        public static TunableNumber MAX_FORWARD_SPEED;
        public static TunableNumber MAX_LATERAL_SPEED;
        public static TunableNumber MAX_ROTATION_SPEED;

        // Tolerances
        public static TunableNumber FORWARD_TOLERANCE;
        public static TunableNumber LATERAL_TOLERANCE;
        public static TunableNumber ROTATION_TOLERANCE;

        // Target distance
        public static TunableNumber TARGET_DISTANCE;

        static void initialize() {
            // Forward PID
            FORWARD_KP = new TunableNumber(TAB_NAME, "Forward kP", 1.0, 0, 0, 2, 1);
            FORWARD_KI = new TunableNumber(TAB_NAME, "Forward kI", 0.0, 0, 1, 2, 1);
            FORWARD_KD = new TunableNumber(TAB_NAME, "Forward kD", 0.0, 0, 2, 2, 1);

            // Lateral PID
            LATERAL_KP = new TunableNumber(TAB_NAME, "Lateral kP", 0.3, 2, 0, 2, 1);
            LATERAL_KI = new TunableNumber(TAB_NAME, "Lateral kI", 0.0, 2, 1, 2, 1);
            LATERAL_KD = new TunableNumber(TAB_NAME, "Lateral kD", 0.0, 2, 2, 2, 1);

            // Rotation PID
            ROTATION_KP = new TunableNumber(TAB_NAME, "Rotation kP", 0.08, 4, 0, 2, 1);
            ROTATION_KI = new TunableNumber(TAB_NAME, "Rotation kI", 0.0, 4, 1, 2, 1);
            ROTATION_KD = new TunableNumber(TAB_NAME, "Rotation kD", 0.0, 4, 2, 2, 1);

            // Speed limits
            MAX_FORWARD_SPEED = new TunableNumber(TAB_NAME, "Max Fwd Speed", 0.625, 6, 0, 2, 1);
            MAX_LATERAL_SPEED = new TunableNumber(TAB_NAME, "Max Lat Speed", 0.625, 6, 1, 2, 1);
            MAX_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Max Rot Speed", 0.9375, 6, 2, 2, 1);

            // Tolerances and target
            FORWARD_TOLERANCE = new TunableNumber(TAB_NAME, "Fwd Tol (m)", 0.10, 8, 0, 2, 1);
            LATERAL_TOLERANCE = new TunableNumber(TAB_NAME, "Lat Tol (m)", 0.05, 8, 1, 2, 1);
            ROTATION_TOLERANCE = new TunableNumber(TAB_NAME, "Rot Tol (deg)", 2.0, 8, 2, 2, 1);
            TARGET_DISTANCE = new TunableNumber(TAB_NAME, "Target Dist (m)", 1.0, 8, 3, 2, 1);
        }
    }

    // ============================================================================
    // MODEL A: ROTATION ONLY
    // ============================================================================

    public static class ModelA {
        private static final String TAB_NAME = "Vision-ModelA";

        public static TunableNumber ROTATION_KP;
        public static TunableNumber ROTATION_KI;
        public static TunableNumber ROTATION_KD;
        public static TunableNumber MAX_ROTATION_SPEED;
        public static TunableNumber MIN_ROTATION_SPEED;
        public static TunableNumber ROTATION_TOLERANCE;

        static void initialize() {
            // Column 0: Rotation PID
            ROTATION_KP = new TunableNumber(TAB_NAME, "Rotation kP", VisionConstants.ModelA.ROTATION_KP, 0, 0, 2, 1);
            ROTATION_KI = new TunableNumber(TAB_NAME, "Rotation kI", VisionConstants.ModelA.ROTATION_KI, 0, 1, 2, 1);
            ROTATION_KD = new TunableNumber(TAB_NAME, "Rotation kD", VisionConstants.ModelA.ROTATION_KD, 0, 2, 2, 1);

            // Column 3: Speed limits and tolerance
            MAX_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Max Rot Speed", VisionConstants.ModelA.MAX_ROTATION_SPEED_RADPS, 3, 0, 2, 1);
            MIN_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Min Rot Speed", VisionConstants.ModelA.MIN_ROTATION_SPEED_RADPS, 3, 1, 2, 1);
            ROTATION_TOLERANCE = new TunableNumber(TAB_NAME, "Rot Tol (deg)", VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES, 3, 2, 2, 1);
        }

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
        private static final String TAB_NAME = "Vision-ModelB";

        // Rotation PID
        public static TunableNumber ROTATION_KP;
        public static TunableNumber ROTATION_KI;
        public static TunableNumber ROTATION_KD;

        // Range PID
        public static TunableNumber RANGE_KP;
        public static TunableNumber RANGE_KI;
        public static TunableNumber RANGE_KD;

        // Speed limits
        public static TunableNumber MAX_ROTATION_SPEED;
        public static TunableNumber MAX_FORWARD_SPEED;

        // Tolerances
        public static TunableNumber ROTATION_TOLERANCE;
        public static TunableNumber DISTANCE_TOLERANCE;

        // Target distance
        public static TunableNumber TARGET_DISTANCE;

        static void initialize() {
            // Column 0: Rotation PID
            ROTATION_KP = new TunableNumber(TAB_NAME, "Rotation kP", VisionConstants.ModelB.ROTATION_KP, 0, 0, 2, 1);
            ROTATION_KI = new TunableNumber(TAB_NAME, "Rotation kI", VisionConstants.ModelB.ROTATION_KI, 0, 1, 2, 1);
            ROTATION_KD = new TunableNumber(TAB_NAME, "Rotation kD", VisionConstants.ModelB.ROTATION_KD, 0, 2, 2, 1);

            // Column 2: Range PID
            RANGE_KP = new TunableNumber(TAB_NAME, "Range kP", VisionConstants.ModelB.RANGE_KP, 2, 0, 2, 1);
            RANGE_KI = new TunableNumber(TAB_NAME, "Range kI", VisionConstants.ModelB.RANGE_KI, 2, 1, 2, 1);
            RANGE_KD = new TunableNumber(TAB_NAME, "Range kD", VisionConstants.ModelB.RANGE_KD, 2, 2, 2, 1);

            // Column 4: Speed limits
            MAX_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Max Rot Speed", VisionConstants.ModelB.MAX_ROTATION_SPEED_RADPS, 4, 0, 2, 1);
            MAX_FORWARD_SPEED = new TunableNumber(TAB_NAME, "Max Fwd Speed", VisionConstants.ModelB.MAX_FORWARD_SPEED_MPS, 4, 1, 2, 1);

            // Column 6: Tolerances and target
            ROTATION_TOLERANCE = new TunableNumber(TAB_NAME, "Rot Tol (deg)", VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES, 6, 0, 2, 1);
            DISTANCE_TOLERANCE = new TunableNumber(TAB_NAME, "Dist Tol (m)", VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS, 6, 1, 2, 1);
            TARGET_DISTANCE = new TunableNumber(TAB_NAME, "Target Dist (m)", VisionConstants.DEFAULT_TARGET_DISTANCE_METERS, 6, 2, 2, 1);
        }

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
        private static final String TAB_NAME = "Vision-ModelC";

        // Rotation PID
        public static TunableNumber ROTATION_KP;
        public static TunableNumber ROTATION_KI;
        public static TunableNumber ROTATION_KD;

        // Range PID
        public static TunableNumber RANGE_KP;
        public static TunableNumber RANGE_KI;
        public static TunableNumber RANGE_KD;

        // Lateral PID
        public static TunableNumber LATERAL_KP;
        public static TunableNumber LATERAL_KI;
        public static TunableNumber LATERAL_KD;

        // Speed limits
        public static TunableNumber MAX_ROTATION_SPEED;
        public static TunableNumber MAX_FORWARD_SPEED;
        public static TunableNumber MAX_LATERAL_SPEED;

        // Tolerances
        public static TunableNumber ROTATION_TOLERANCE;
        public static TunableNumber DISTANCE_TOLERANCE;
        public static TunableNumber LATERAL_TOLERANCE;

        // Lateral deadband
        public static TunableNumber LATERAL_DEADBAND;

        // Target distance
        public static TunableNumber TARGET_DISTANCE;

        static void initialize() {
            // Column 0: Rotation PID
            ROTATION_KP = new TunableNumber(TAB_NAME, "Rotation kP", VisionConstants.ModelC.ROTATION_KP, 0, 0, 2, 1);
            ROTATION_KI = new TunableNumber(TAB_NAME, "Rotation kI", VisionConstants.ModelC.ROTATION_KI, 0, 1, 2, 1);
            ROTATION_KD = new TunableNumber(TAB_NAME, "Rotation kD", VisionConstants.ModelC.ROTATION_KD, 0, 2, 2, 1);

            // Column 2: Range PID
            RANGE_KP = new TunableNumber(TAB_NAME, "Range kP", VisionConstants.ModelC.RANGE_KP, 2, 0, 2, 1);
            RANGE_KI = new TunableNumber(TAB_NAME, "Range kI", VisionConstants.ModelC.RANGE_KI, 2, 1, 2, 1);
            RANGE_KD = new TunableNumber(TAB_NAME, "Range kD", VisionConstants.ModelC.RANGE_KD, 2, 2, 2, 1);

            // Column 4: Lateral PID
            LATERAL_KP = new TunableNumber(TAB_NAME, "Lateral kP", VisionConstants.ModelC.LATERAL_KP, 4, 0, 2, 1);
            LATERAL_KI = new TunableNumber(TAB_NAME, "Lateral kI", VisionConstants.ModelC.LATERAL_KI, 4, 1, 2, 1);
            LATERAL_KD = new TunableNumber(TAB_NAME, "Lateral kD", VisionConstants.ModelC.LATERAL_KD, 4, 2, 2, 1);

            // Column 6: Speed limits
            MAX_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Max Rot Speed", VisionConstants.ModelC.MAX_ROTATION_SPEED_RADPS, 6, 0, 2, 1);
            MAX_FORWARD_SPEED = new TunableNumber(TAB_NAME, "Max Fwd Speed", VisionConstants.ModelC.MAX_FORWARD_SPEED_MPS, 6, 1, 2, 1);
            MAX_LATERAL_SPEED = new TunableNumber(TAB_NAME, "Max Lat Speed", VisionConstants.ModelC.MAX_LATERAL_SPEED_MPS, 6, 2, 2, 1);

            // Column 8: Tolerances, deadband, and target
            ROTATION_TOLERANCE = new TunableNumber(TAB_NAME, "Rot Tol (deg)", VisionConstants.ModelC.ROTATION_TOLERANCE_DEGREES, 8, 0, 2, 1);
            DISTANCE_TOLERANCE = new TunableNumber(TAB_NAME, "Dist Tol (m)", VisionConstants.ModelC.DISTANCE_TOLERANCE_METERS, 8, 1, 2, 1);
            LATERAL_TOLERANCE = new TunableNumber(TAB_NAME, "Lat Tol (deg)", VisionConstants.ModelC.LATERAL_TOLERANCE_DEGREES, 8, 2, 2, 1);
            LATERAL_DEADBAND = new TunableNumber(TAB_NAME, "Lat Deadband", VisionConstants.ModelC.LATERAL_DEADBAND_DEGREES, 8, 3, 2, 1);
            TARGET_DISTANCE = new TunableNumber(TAB_NAME, "Target Dist (m)", VisionConstants.DEFAULT_TARGET_DISTANCE_METERS, 8, 4, 2, 1);
        }
    }

    // ============================================================================
    // MODEL D: COLOR BLOB HUNT
    // ============================================================================

    public static class ModelD {
        private static final String TAB_NAME = "Vision-ModelD";

        // Hunt mode PID
        public static TunableNumber HUNT_ROTATION_KP;
        public static TunableNumber HUNT_ROTATION_KI;
        public static TunableNumber HUNT_ROTATION_KD;

        // Seek mode rotation PID
        public static TunableNumber SEEK_ROTATION_KP;
        public static TunableNumber SEEK_ROTATION_KI;
        public static TunableNumber SEEK_ROTATION_KD;

        // Range PID
        public static TunableNumber RANGE_KP;
        public static TunableNumber RANGE_KI;
        public static TunableNumber RANGE_KD;

        // Speed limits
        public static TunableNumber HUNT_ROTATION_SPEED;
        public static TunableNumber MAX_ROTATION_SPEED;
        public static TunableNumber MAX_FORWARD_SPEED;

        // Tolerances
        public static TunableNumber ROTATION_TOLERANCE;
        public static TunableNumber DISTANCE_TOLERANCE;

        // Color blob thresholds
        public static TunableNumber MIN_TARGET_AREA;
        public static TunableNumber TARGET_AREA_FOR_DISTANCE;

        static void initialize() {
            // Column 0: Hunt Rotation PID
            HUNT_ROTATION_KP = new TunableNumber(TAB_NAME, "Hunt Rot kP", VisionConstants.ModelD.HUNT_ROTATION_KP, 0, 0, 2, 1);
            HUNT_ROTATION_KI = new TunableNumber(TAB_NAME, "Hunt Rot kI", VisionConstants.ModelD.HUNT_ROTATION_KI, 0, 1, 2, 1);
            HUNT_ROTATION_KD = new TunableNumber(TAB_NAME, "Hunt Rot kD", VisionConstants.ModelD.HUNT_ROTATION_KD, 0, 2, 2, 1);

            // Column 2: Seek Rotation PID
            SEEK_ROTATION_KP = new TunableNumber(TAB_NAME, "Seek Rot kP", VisionConstants.ModelD.SEEK_ROTATION_KP, 2, 0, 2, 1);
            SEEK_ROTATION_KI = new TunableNumber(TAB_NAME, "Seek Rot kI", VisionConstants.ModelD.SEEK_ROTATION_KI, 2, 1, 2, 1);
            SEEK_ROTATION_KD = new TunableNumber(TAB_NAME, "Seek Rot kD", VisionConstants.ModelD.SEEK_ROTATION_KD, 2, 2, 2, 1);

            // Column 4: Range PID
            RANGE_KP = new TunableNumber(TAB_NAME, "Range kP", VisionConstants.ModelD.RANGE_KP, 4, 0, 2, 1);
            RANGE_KI = new TunableNumber(TAB_NAME, "Range kI", VisionConstants.ModelD.RANGE_KI, 4, 1, 2, 1);
            RANGE_KD = new TunableNumber(TAB_NAME, "Range kD", VisionConstants.ModelD.RANGE_KD, 4, 2, 2, 1);

            // Column 6: Speed limits
            HUNT_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Hunt Rot Speed", VisionConstants.ModelD.HUNT_ROTATION_SPEED_RADPS, 6, 0, 2, 1);
            MAX_ROTATION_SPEED = new TunableNumber(TAB_NAME, "Max Rot Speed", VisionConstants.ModelD.MAX_ROTATION_SPEED_RADPS, 6, 1, 2, 1);
            MAX_FORWARD_SPEED = new TunableNumber(TAB_NAME, "Max Fwd Speed", VisionConstants.ModelD.MAX_FORWARD_SPEED_MPS, 6, 2, 2, 1);

            // Column 8: Tolerances and blob thresholds
            ROTATION_TOLERANCE = new TunableNumber(TAB_NAME, "Rot Tol (deg)", VisionConstants.ModelD.ROTATION_TOLERANCE_DEGREES, 8, 0, 2, 1);
            DISTANCE_TOLERANCE = new TunableNumber(TAB_NAME, "Dist Tol (m)", VisionConstants.ModelD.DISTANCE_TOLERANCE_METERS, 8, 1, 2, 1);
            MIN_TARGET_AREA = new TunableNumber(TAB_NAME, "Min Target Area", VisionConstants.ModelD.MIN_TARGET_AREA, 8, 2, 2, 1);
            TARGET_AREA_FOR_DISTANCE = new TunableNumber(TAB_NAME, "Target Area Dist", VisionConstants.ModelD.TARGET_AREA_FOR_DISTANCE, 8, 3, 2, 1);
        }
    }

    // ============================================================================
    // CAMERA CONFIGURATION (less frequently tuned, but available)
    // ============================================================================

    public static class Camera {
        private static final String TAB_NAME = "Vision-Camera";

        public static TunableNumber HEIGHT;
        public static TunableNumber ANGLE;
        public static TunableNumber LATERAL_OFFSET;
        public static TunableNumber LONGITUDINAL_OFFSET;

        static void initialize() {
            HEIGHT = new TunableNumber(TAB_NAME, "Height (m)", VisionConstants.CAMERA_HEIGHT_METERS, 0, 0, 2, 1);
            ANGLE = new TunableNumber(TAB_NAME, "Angle (deg)", VisionConstants.CAMERA_ANGLE_DEGREES, 0, 1, 2, 1);
            LATERAL_OFFSET = new TunableNumber(TAB_NAME, "Lateral Offset (m)", VisionConstants.CAMERA_LATERAL_OFFSET_METERS, 0, 2, 2, 1);
            LONGITUDINAL_OFFSET = new TunableNumber(TAB_NAME, "Long Offset (m)", VisionConstants.CAMERA_LONGITUDINAL_OFFSET_METERS, 0, 3, 2, 1);
        }
    }

    /**
     * Initialize all tunable constants with Shuffleboard tabs and widgets.
     * MUST be called during robot initialization to create all widgets.
     */
    public static void initializeAll() {
        if (initialized) {
            return;
        }

        Main.initialize();
        ModelA.initialize();
        ModelB.initialize();
        ModelC.initialize();
        ModelD.initialize();
        Camera.initialize();

        initialized = true;
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

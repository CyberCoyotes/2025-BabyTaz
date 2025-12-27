package frc.robot.subsystems.vision;

/**
 * Constants for Vision Alignment Test Models.
 *
 * This file centralizes all tuning parameters for the vision test commands,
 * making it easy to adjust values across all models from one location.
 *
 * Test Models Overview:
 * - Model A: Rotational alignment only (robot rotates to center AprilTag on Y-axis)
 * - Model B: Rotational alignment + X-axis distance (rotate + drive to target distance)
 * - Model C: Perpendicular alignment with distance (face tag perpendicular + distance)
 * - Model D: Color blob hunt and seek (teal color detection with range alignment)
 */
public final class VisionTestConstants {

    private VisionTestConstants() {} // Prevent instantiation

    // ============================================================================
    // COMMON TARGET VALUES
    // ============================================================================

    /** Default target distance from AprilTag in meters (X-axis) */
    public static final double DEFAULT_TARGET_DISTANCE_METERS = 1.2;

    /** Target horizontal offset - 0 means centered on robot Y-axis */
    public static final double TARGET_TX_CENTERED = 0.0;

    // ============================================================================
    // CAMERA CONFIGURATION (from VisionConstants)
    // ============================================================================

    /** Camera height from floor in meters */
    public static final double CAMERA_HEIGHT_METERS = VisionConstants.CAMERA_HEIGHT_METERS;

    /** Camera mounting angle in degrees (0 = level) */
    public static final double CAMERA_ANGLE_DEGREES = VisionConstants.CAMERA_ANGLE_DEGREES;

    /** AprilTag center height from floor in meters */
    public static final double TAG_HEIGHT_METERS = VisionConstants.TAG_HEIGHT_METERS;

    // ============================================================================
    // MODEL A: ROTATIONAL ALIGNMENT ONLY
    // Limelight Example: "Aiming with Servoing"
    // Robot rotates to center AprilTag on Y-axis (tx = 0)
    // ============================================================================

    public static final class ModelA {
        // PID Tuning for rotation
        public static final double ROTATION_KP = 0.035;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        // Speed limits
        public static final double MAX_ROTATION_SPEED_RADPS = 1.0;

        // Tolerance
        public static final double ROTATION_TOLERANCE_DEGREES = 1.5;

        // Minimum angular velocity to overcome friction (servoing)
        public static final double MIN_ROTATION_SPEED_RADPS = 0.05;
    }

    // ============================================================================
    // MODEL B: ROTATIONAL ALIGNMENT + X-AXIS DISTANCE
    // Limelight Example: "Aiming and Ranging Simultaneously"
    // Robot rotates to center tag AND drives to target distance
    // ============================================================================

    public static final class ModelB {
        // Rotation PID
        public static final double ROTATION_KP = 0.035;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        // Forward/Range PID
        public static final double RANGE_KP = 1.0;
        public static final double RANGE_KI = 0.0;
        public static final double RANGE_KD = 0.05;

        // Speed limits
        public static final double MAX_ROTATION_SPEED_RADPS = 1.0;
        public static final double MAX_FORWARD_SPEED_MPS = 0.8;

        // Tolerances
        public static final double ROTATION_TOLERANCE_DEGREES = 1.5;
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;
    }

    // ============================================================================
    // MODEL C: PERPENDICULAR ALIGNMENT WITH DISTANCE
    // Limelight Example: "Aiming and Ranging with Swerve"
    // Robot aligns perpendicular to tag face while maintaining target distance
    // Uses MegaTag2 for pose data when available
    // ============================================================================

    public static final class ModelC {
        // Rotation PID (perpendicular to tag)
        public static final double ROTATION_KP = 0.05;
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.0;

        // Forward/Range PID (X-axis distance)
        public static final double RANGE_KP = 1.2;
        public static final double RANGE_KI = 0.0;
        public static final double RANGE_KD = 0.1;

        // Lateral/Strafe PID (Y-axis centering)
        public static final double LATERAL_KP = 0.04;
        public static final double LATERAL_KI = 0.0;
        public static final double LATERAL_KD = 0.0;

        // Speed limits
        public static final double MAX_ROTATION_SPEED_RADPS = 1.0;
        public static final double MAX_FORWARD_SPEED_MPS = 0.8;
        public static final double MAX_LATERAL_SPEED_MPS = 0.5;

        // Tolerances
        public static final double ROTATION_TOLERANCE_DEGREES = 2.0;
        public static final double DISTANCE_TOLERANCE_METERS = 0.05;
        public static final double LATERAL_TOLERANCE_DEGREES = 2.0;

        // Deadband for lateral movement (prevents small corrections causing drift)
        public static final double LATERAL_DEADBAND_DEGREES = 3.0;
    }

    // ============================================================================
    // MODEL D: COLOR BLOB HUNT AND SEEK
    // Limelight Example: "Getting in Range" with color pipeline
    // Hunt mode: Rotate to find teal color blob
    // Seek mode: Drive to target distance while centered on blob
    // ============================================================================

    public static final class ModelD {
        // Hunt mode - rotation to find target
        public static final double HUNT_ROTATION_KP = 0.03;
        public static final double HUNT_ROTATION_KI = 0.0;
        public static final double HUNT_ROTATION_KD = 0.0;

        // Seek mode - rotation to stay centered
        public static final double SEEK_ROTATION_KP = 0.035;
        public static final double SEEK_ROTATION_KI = 0.0;
        public static final double SEEK_ROTATION_KD = 0.0;

        // Range PID for seek mode
        public static final double RANGE_KP = 0.8;
        public static final double RANGE_KI = 0.0;
        public static final double RANGE_KD = 0.05;

        // Speed limits - hunt mode (slower rotation while searching)
        public static final double HUNT_ROTATION_SPEED_RADPS = 0.5;

        // Speed limits - seek mode
        public static final double MAX_ROTATION_SPEED_RADPS = 1.0;
        public static final double MAX_FORWARD_SPEED_MPS = 0.6;

        // Tolerances
        public static final double ROTATION_TOLERANCE_DEGREES = 3.0;
        public static final double DISTANCE_TOLERANCE_METERS = 0.08;

        // Color blob detection thresholds
        public static final double MIN_TARGET_AREA = 0.1;  // Minimum area percentage to consider valid
        public static final double TARGET_AREA_FOR_DISTANCE = 3.0;  // Expected area at 1.2m distance (tunable)

        // Pipeline index for color blob detection (configure in Limelight web interface)
        public static final int COLOR_PIPELINE_INDEX = 1;

        // AprilTag pipeline for fallback
        public static final int APRILTAG_PIPELINE_INDEX = 0;
    }

    // ============================================================================
    // SHUFFLEBOARD CONFIGURATION
    // ============================================================================

    public static final class Dashboard {
        public static final String TAB_NAME = "Vision Tests";
        public static final String MODEL_A_BUTTON = "Model A: Rotation Only";
        public static final String MODEL_B_BUTTON = "Model B: Rotation + Range";
        public static final String MODEL_C_BUTTON = "Model C: Perpendicular + Range";
        public static final String MODEL_D_BUTTON = "Model D: Color Hunt & Seek";
        public static final String STOP_BUTTON = "STOP ALL";
    }

    // ============================================================================
    // NETWORK TABLES KEYS (for telemetry output)
    // ============================================================================

    public static final class NTKeys {
        public static final String MODEL_A_PREFIX = "VisionTest/ModelA/";
        public static final String MODEL_B_PREFIX = "VisionTest/ModelB/";
        public static final String MODEL_C_PREFIX = "VisionTest/ModelC/";
        public static final String MODEL_D_PREFIX = "VisionTest/ModelD/";
    }
}

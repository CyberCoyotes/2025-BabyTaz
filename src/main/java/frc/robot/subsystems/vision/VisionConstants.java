package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

/**
 * Constants for Vision Alignment.
 *
 * Includes configurations for:
 * - Camera physical setup
 * - Vision alignment test models (A, B, C, D)
 * - PID tuning parameters
 * - Motion constraints
 */
public final class VisionConstants {

    private VisionConstants() {} // Prevent instantiation

    // ============================================================================
    // CAMERA CONFIGURATION
    // ============================================================================

    /** Camera height from floor in meters */
    public static final double CAMERA_HEIGHT_METERS = 0.49784;  // 49.784 cm (19.6") from floor to camera lens

    /** Camera mounting angle in degrees (0 = level, positive = tilted up) */
    public static final double CAMERA_ANGLE_DEGREES = 0.0;

    /**
     * Camera lateral offset from robot center in meters.
     * Positive = camera is to the right of center
     * Negative = camera is to the left of center
     * 0 = camera is centered on robot's Y-axis
     */
    public static final double CAMERA_LATERAL_OFFSET_METERS = 0.10;  // 10 cm to the right of center

    /**
     * Camera longitudinal offset from robot center in meters.
     * Positive = camera is forward of center
     * Negative = camera is behind center
     * 0 = camera is centered on robot's X-axis
     */
    public static final double CAMERA_LONGITUDINAL_OFFSET_METERS = 0.10;  // 10 cm forward of center (near front bumper)

    /** AprilTag center height from floor in meters */
    public static final double TAG_HEIGHT_METERS = 0.243;

    // ============================================================================
    // PHYSICAL MOUNT CONFIGURATION
    // ============================================================================

    /** Camera is mounted on front of robot (same side as Pigeon 2 X-axis) */
    public static final boolean MOUNTED_ON_FRONT = true;

    /**
     * Direction multiplier for Limelight movements based on mount position.
     * -1.0 for front mount, 1.0 for back mount.
     */
    public static final double LIMELIGHT_DIRECTION = MOUNTED_ON_FRONT ? -1.0 : 1.0;

    /**
     * Rotation direction multiplier for vision alignment.
     * For front-mounted camera: -1.0 (negates PID output)
     * This accounts for the camera orientation relative to robot rotation.
     */
    public static final double ROTATION_DIRECTION_MULTIPLIER = -1.0;

    // ============================================================================
    // APRILTAG VALIDATION
    // ============================================================================

    public static final int MIN_VALID_TAG = 1;
    public static final int MAX_VALID_TAG = 22;

    public static boolean isValidTagId(int tagId) {
        return tagId >= MIN_VALID_TAG && tagId <= MAX_VALID_TAG;
    }

    // ============================================================================
    // COMMON TARGET VALUES
    // ============================================================================

    /** Default target distance from AprilTag in meters (X-axis) */
    public static final double DEFAULT_TARGET_DISTANCE_METERS = 0.75;

    /** Target horizontal offset - 0 means centered on robot Y-axis */
    public static final double TARGET_TX_CENTERED = 0.0;

    public static final double LEFT_TARGET_OFFSET = -15.8; // TODO Change back when printed properly 10 cm left offset
    public static final double CENTER_TARGET_OFFSET = 0.0;  // No offset
    public static final double RIGHT_TARGET_OFFSET = 15.8; // TODO 10 cm right offset

    // ============================================================================
    // VISION TEST MODELS
    // Test Models Overview:
    // - Model A: Rotational alignment only (robot rotates to center AprilTag on Y-axis)
    // - Model B: Rotational alignment + X-axis distance (rotate + drive to target distance)
    // - Model C: Perpendicular alignment with distance (face tag perpendicular + distance)
    // - Model D: Color blob hunt and seek (teal color detection with range alignment)
    // ============================================================================

    // ============================================================================
    // MODEL A: ROTATIONAL ALIGNMENT ONLY
    // Limelight Example: "Aiming with Servoing"
    // Robot rotates to center AprilTag on Y-axis (tx = 0)
    // ============================================================================

    public static final class ModelA {
        // PID Tuning for rotation
        public static final double ROTATION_KP = 0.04;  // Increased from 0.035 to provide more power near target
        public static final double ROTATION_KI = 0.0;
        public static final double ROTATION_KD = 0.002;  // Added small D to help push through to target

        // Speed limits
        public static final double MAX_ROTATION_SPEED_RADPS = 2.0;

        // Tolerance
        public static final double ROTATION_TOLERANCE_DEGREES = 1.5;

        // Minimum angular velocity to overcome friction (servoing)
        public static final double MIN_ROTATION_SPEED_RADPS = 0.08;  // Increased from 0.05 to overcome static friction
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
    // MODEL C: PERPENDICULAR ALIGNMENT WITH DISTANCE (Full Align)
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

    // Motion constraints
    public static final double MAX_TRANSLATIONAL_VELOCITY = 0.1; 
        /* 
        I honestly don't think these impact the robot's performance.
        |   0.5 |
        |   0.25 |
        |   0.1  |
        */
        public static final double MAX_TRANSLATIONAL_ACCELERATION = 0.25;
        /* 
        I honestly don't think these impact the robot's performance either
        |   1.0     |
        |   0.25    |
        */
        
        public static final double MAX_ANGULAR_VELOCITY = Math.PI/2;
        public static final double MAX_ANGULAR_ACCELERATION = Math.PI;
    
        // Tolerances  
        public static final double POSITION_TOLERANCE = 0.02; // m
        public static final double ANGULAR_TOLERANCE = 0.05; // radians
        // src\main\java\frc\robot\documentation\degrees-to-radians.md
            
        // PID gains
        public static double TRANSLATIONAL_kP = 0.25;
        // 0.4 aggressive
        // 0.1 jittery
        // 0.2 jittery but less
        // 0.3 too aggressive
        // 0.25 Seems ok, but overshoots a little

        public static double TRANSLATIONAL_kI = 0.0; 
        public static double TRANSLATIONAL_kD = 0.05;
        //  |   0.25    |   0.05    |   solid results
        //  |   0.20    |   0.05    |   jittery or overshoot?
        //  |   0.20    |   0.10    |   worse
        //  |   0.25    |   
        public static double ANGULAR_kP = 0.5;
        public static double ANGULAR_kI = 0.0;
        public static double ANGULAR_kD = 0.0;

        // When inputing from joystick + left OR rightbumper, ignore velocityX
    
        // CTRE Motion parameters for rotation and translation
        public static final double ROT_VELOCITY = 80.0;  
        public  static final double ROT_ACCEL = 160.0;    
        public  static final double ROT_JERK = 1600.0;    
        public  static final double TRANS_VELOCITY = 2.0; // m/s  
        public  static final double TRANS_ACCEL = 4.0;    // m/s^2
        public  static final double TRANS_JERK = 8.0;     // m/s^3
        // Vision processing constants
        public static final double MIN_TARGET_AREA = 0.1;  // Minimum target area to be valid
        public static final double MAX_TARGET_DISTANCE = 2.0; // Maximum valid distance in meters
    
        /* 
    // Physical mounting configuration
    public static final boolean LIMELIGHT_MOUNTED_ON_FRONT = false;
    
    // Target parameters
    public static final double TARGET_DISTANCE_METERS = 0.5;
    
    The value of LIMELIGHT_DIRECTION_MULTIPLIER is determined using a ternary conditional operator (? :). This operator evaluates the boolean expression LIMELIGHT_MOUNTED_ON_FRONT. If LIMELIGHT_MOUNTED_ON_FRONT is true, the constant is assigned a value of -1.0. 
    If LIMELIGHT_MOUNTED_ON_FRONT is false, the constant is assigned a value of 1.0.
    This pattern is used in scenarios where the direction or orientation of a component (in this case, a Limelight camera) affects calculations or logic. 
    By using the LIMELIGHT_DIRECTION_MULTIPLIER,
    the code can easily adjust for whether the Limelight is mounted on the front or another position, 
    ensuring that directional calculations remain consistent and correct. 
    
    public static final double LIMELIGHT_DIRECTION = LIMELIGHT_MOUNTED_ON_FRONT ? 1.0 : -1.0;
    
    // Vision processing constants
    public static final double MIN_TARGET_AREA = 0.1;  // Minimum target area to be valid
    public static final double MAX_TARGET_DISTANCE = 2.0; // Maximum valid distance in meters
    
    // AprilTag validation ranges
    public static final int MIN_VALID_TAG = 1;  // Minimum valid AprilTag ID
    public static final int MAX_VALID_TAG = 22; // Maximum valid AprilTag ID

    // PID & Control constants 
    public static final double ALIGN_TRANSLATION_P = 1.0;
    public static final double ALIGN_ROTATION_P = 0.05;
    // public static final double TARGET_DISTANCE = 0.5; // meters

    // Motion constraints
    public static final double MAX_TRANSLATION_VELOCITY = .5; // meters per second
    public static final double MAX_TRANSLATION_ACCELERATION = 0.2; // meters per second squared
    public static final double MAX_ROTATION_VELOCITY = Math.PI; // radians per second
    public static final double MAX_ROTATION_ACCELERATION = Math.PI; // radians per second squared

    public static final double VISION_kP = 0.035; // Proportional control for vision steering
    public static final double VISION_kI = 0.0;
    public static final double VISION_kD = 0.0;

    // PID gains
    public static final double TRANSLATION_kP = 0.4;
    public static final double TRANSLATION_kI = 0.0;
    public static final double TRANSLATION_kD = 0.05;

    public static final double ROTATION_kP = 0.5;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;

    // Tolerances
    public static final double TRANSLATION_TOLERANCE_METERS = 0.04; // 4 cm tolerance
    public static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(2.0);
        
    public static boolean isValidTagId(int tagId) {
        return tagId >= MIN_VALID_TAG && tagId <= MAX_VALID_TAG;
    }
    
    public static boolean isValidTargetArea(double area) {
        return area >= MIN_TARGET_AREA;
    }
    
        // Swerve Drivetrain 
    public static final double MAX_SPEED = 4.5; // meters per second
    public static final double MAX_ANGULAR_RATE = Math.PI; // radians per second

    // Deadbands prevent drift from tiny joystick movements
    public static final double DRIVE_DEADBAND = 0.1; 
    public static final double ROTATION_DEADBAND = 0.1;

    // Movement thresholds
    public static final double SPEED_THRESHOLD = 0.05; // m/s - Below this speed, robot can coast
    public static final double COAST_TIMEOUT = 5.0; // seconds before allowing coast mode
    
    // Auto-rotate settings
    public static final double ROTATE_TOLERANCE = Units.degreesToRadians(1.5);
    public static final double MIN_ROTATE_SPEED = Units.degreesToRadians(4);
    */
    

}
package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

public final class VisionConstants {
        // Physical mount config
        /* Value is determined using a ternary conditional operator (? :). 
        This operator evaluates the MOUNTED_ON_FRONT boolean.
        If MOUNTED_ON_FRONT is true, LIMELIGHT_DIRECTION is assigned a value of 1.0. 
        If MOUNTED_ON_FRONT is false, LIMELIGHT_DIRECTION is assigned a value of -1.0.
        */
        public static final boolean MOUNTED_ON_FRONT = false;
        public static final double LIMELIGHT_DIRECTION = MOUNTED_ON_FRONT ? 1.0 : -1.0;
        
        // AprilTag validation
        public static final int MIN_VALID_TAG = 1;
        public static final int MAX_VALID_TAG = 22;
    
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

        public static double TRANSLATIONAL_kI = 0.0; 
        public static double TRANSLATIONAL_kD = 0.05;
        public static double ANGULAR_kP = 0.5;
        public static double ANGULAR_kI = 0.0;
        public static double ANGULAR_kD = 0.0;
    
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
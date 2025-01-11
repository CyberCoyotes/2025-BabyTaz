package frc.robot.subsystems.vision;

public final class VisionConstants {
    // Physical mounting configuration
    public static final boolean LIMELIGHT_MOUNTED_ON_FRONT = false;
    
    /* The value of LIMELIGHT_DIRECTION_MULTIPLIER is determined using a ternary conditional operator (? :). This operator evaluates the boolean expression LIMELIGHT_MOUNTED_ON_FRONT. If LIMELIGHT_MOUNTED_ON_FRONT is true, the constant is assigned a value of -1.0. 
    If LIMELIGHT_MOUNTED_ON_FRONT is false, the constant is assigned a value of 1.0.
    This pattern is used in scenarios where the direction or orientation of a component (in this case, a Limelight camera) affects calculations or logic. 
    By using the LIMELIGHT_DIRECTION_MULTIPLIER,
    the code can easily adjust for whether the Limelight is mounted on the front or another position, 
    ensuring that directional calculations remain consistent and correct. 
    */
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
    
    public static boolean isValidTagId(int tagId) {
        return tagId >= MIN_VALID_TAG && tagId <= MAX_VALID_TAG;
    }
    
    public static boolean isValidTargetArea(double area) {
        return area >= MIN_TARGET_AREA;
    }
    
}
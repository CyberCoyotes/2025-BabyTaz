package frc.robot.vision20;

public final class VisionConstants {
    // Limelight physical mounting and target configuration (in meters and degrees)
    public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 30.0; // Adjust to your limelight angle
    public static final double LIMELIGHT_HEIGHT_METERS = 0.5;           // Height of your Limelight off the floor
    public static final double TARGET_HEIGHT_METERS = 2.5;              // Height of the apriltag target

    // PID gains for tag alignment (tune these for your robot)
    public static final double ALIGN_P = 0.05;
    public static final double ALIGN_I = 0.0;
    public static final double ALIGN_D = 0.0;

    // Desired horizontal offset from target (in degrees) when aligned
    public static final double ALIGN_TARGET_OFFSET_DEGREES = 0.0;

    // Acceptable error threshold (deadband) for alignment
    public static final double ALIGN_DEADBAND_DEGREES = 1.0;

    // Maximum speeds (tweak these to match your drivetrain capabilities)
    public static final double MAX_ROTATION_SPEED = 0.5; // Maximum rotational speed
    public static final double MAX_FORWARD_SPEED = 0.3;  // Forward speed when approaching the target

    // Example threshold for distance-based adjustments (in meters)
    public static final double APPROACH_DISTANCE_THRESHOLD = 1.0;
}

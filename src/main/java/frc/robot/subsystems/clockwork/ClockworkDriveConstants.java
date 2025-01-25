package frc.robot.subsystems.clockwork;

import edu.wpi.first.math.util.Units;

public final class ClockworkDriveConstants {
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
    public static final double ROTATE_kP = 6.0;
    public static final double ROTATE_kI = 0.0;
    public static final double ROTATE_kD = 0.0;
    public static final double ROTATE_TOLERANCE = Units.degreesToRadians(1.5);
    public static final double MIN_ROTATE_SPEED = Units.degreesToRadians(4);
    
    // Limelight 
    public static final boolean LIMELIGHT_MOUNTED_FRONT = false; // TODO Is Limelight mounted on front
    public static final double VISION_kP = 0.035; // Proportional control for vision steering
    public static final double MIN_TARGET_AREA = 0.1; // Minimum area to be considered valid target
}
package frc.robot.vision19;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    // Limelight mount position and orientation relative to robot center
    public static final Transform3d LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(11.5),  // X forward
            Units.inchesToMeters(-2.5),  // Y left
            Units.inchesToMeters(22.0)   // Z up
        ),
        new Rotation3d(
            Units.degreesToRadians(0),   // Roll
            Units.degreesToRadians(15),  // Pitch (angle up/down)
            Units.degreesToRadians(0)    // Yaw
        )
    );

    // Vision processing constants
    public static final double MIN_TARGET_AREA = 0.1;  // Minimum area % for valid detection
    public static final double MAX_POSE_AMBIGUITY = 0.2;  // Maximum ambiguity score for pose estimation
    public static final double MAX_VALID_RANGE = Units.feetToMeters(20);

    // PID Constants for tag alignment
    public static final double ALIGN_P = 0.03;
    public static final double ALIGN_I = 0.0;
    public static final double ALIGN_D = 0.002;

    // Alignment tolerances
    public static final double ALIGNED_THRESHOLD_X = Units.inchesToMeters(2);
    public static final double ALIGNED_THRESHOLD_Y = Units.inchesToMeters(2);
    public static final double ALIGNED_THRESHOLD_ROTATION = Units.degreesToRadians(2);

    // Rate limits for smooth motion
    public static final double MAX_ANGULAR_VELOCITY = Units.degreesToRadians(180);
    public static final double MAX_DRIVE_VELOCITY = 2.0; // already in meters per second
}
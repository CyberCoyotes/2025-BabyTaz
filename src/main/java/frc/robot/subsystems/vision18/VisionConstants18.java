package frc.robot.subsystems.vision18;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants18 {
// Updated for front-mounted camera
public static final Transform3d ROBOT_TO_CAM = new Transform3d(
    new Translation3d(
        0.2, // Camera is 20cm in front of robot or 20cm from center of robot?
        0.0, // Camera is centered on robot "left-right"
        0.2 // Camera is 20cm above robot center 
      ),
    new Rotation3d(
        0, //
        Math.toRadians(0), //
        Math.toRadians(0)) //
);

    // Vision processing constants
    public static final double MIN_TARGET_AREA = 0.1; // Minimum target area to be considered valid
    public static final double MAX_POSE_AMBIGUITY = 0.15; // Maximum ambiguity score to accept pose
    public static final double MAX_POSE_LATENCY = 0.5; // Maximum latency to accept pose (seconds)
    
    // Standard deviation coefficients for vision measurements
    public static final double XY_STD_DEV_COEFFICIENT = 0.01; 
    public static final double THETA_STD_DEV_COEFFICIENT = 0.01;

    // Distance-based standard deviation scaling
    public static final double MIN_RANGE = 0.5; // Minimum range for full confidence (meters)
    public static final double MAX_RANGE = 4.0; // Maximum range before rejecting (meters)
    
// Adjusted constants
public static final double TARGET_DISTANCE_METERS = 1.0; // Verify this is your desired distance
public static final double DEADBAND_METERS = 0.05; // 5cm deadband
public static final double ROTATION_DEADBAND_DEGREES = Math.toRadians(2.0);
}
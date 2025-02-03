package frc.robot.subsystems.vision18;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants18 {
// Updated for front-mounted camera
public static final Transform3d ROBOT_TO_CAM = new Transform3d(
    new Translation3d(0.5, 0.0, 0.5), // Front mounted - positive X
    new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)) // No rotation needed
);

// Adjusted constants
public static final double TARGET_DISTANCE_METERS = 1.0; // Verify this is your desired distance
public static final double DEADBAND_METERS = 0.05; // 5cm deadband
public static final double ROTATION_DEADBAND_DEGREES = Math.toRadians(2.0);
}
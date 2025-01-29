// VisionConstants.java  (renamed from VisionConstantsV15.java)
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public final class VisionConstantsV15 {
    // Camera mounted on back of robot, 30 inches up, 5 inches back from center
    public static final Transform3d CAMERA_POSITION = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(-5.0),  // X (negative because it's behind center)
            Units.inchesToMeters(0.0),   // Y (centered)
            Units.inchesToMeters(30.0)   // Z (height off ground)
        ),
        new Rotation3d(0.0, 0.0, Math.PI) // Rotated 180 degrees (facing backward)
    );

    // Vision validation constants
    public static final double AMBIGUITY_THRESHOLD = 0.15;
    public static final double MIN_TARGET_AREA = 0.1;
    public static final double MAX_VALID_DISTANCE = 4.0;
    public static final double POSE_HISTORY_SECONDS = 0.1;

    // Target alignment constants
    public static final double TARGET_DISTANCE_METERS = 0.10;
    public static final double TRANSLATION_TOLERANCE_METERS = 0.02;
    public static final double ANGLE_TOLERANCE_DEGREES = 5.0;
}
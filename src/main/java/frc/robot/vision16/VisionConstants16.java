package frc.robot.vision16;

public class VisionConstants16 {
    // Constants for distance calculation
    public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0.0;  // Angle of limelight mount from horizontal
    public static final double LIMELIGHT_MOUNT_HEIGHT_METERS = 0.305;   // Height of limelight lens from ground
   
    /* 6 7/8 inches to meters */
    public static final double REEF_TARGET_HEIGHT = 0.174625;           // Height of AprilTag center from ground
     
    // For reference: 2024 AprilTag heights
    // Source stage tags: 1.35 to 1.45 meters
    // Amp tags: 1.22 meters
    // Speaker tags: 1.42 to 1.56 meters

    // Difference between camera mount and ToF mount is 3.5"
}

// 6 7/8 in from the ground




// Find the distance with Limelight
// Compare to the distance returned by the PlayinWithFusion laser

// public static double tagDistance = (targetHeight - cameraHeight) / tan(cameraAngle + targetY)
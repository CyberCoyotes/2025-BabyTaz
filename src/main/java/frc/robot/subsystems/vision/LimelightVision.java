package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * Simple vision subsystem for AprilTag detection using Limelight.
 * Provides basic target detection and measurement data.
 */
public class LimelightVision extends SubsystemBase {
    private final String limelightName;
    private final NetworkTable limelightTable;

    public LimelightVision(String limelightName) {
        this.limelightName = limelightName;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // Set to pipeline 0 for AprilTag detection
        LimelightHelpers.setPipelineIndex(limelightName, 0);
    }

    /**
     * Check if a valid AprilTag target is visible.
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Get the AprilTag ID (fiducial ID).
     */
    public int getTagID() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    /**
     * Get horizontal offset to target in degrees.
     * Positive means target is to the right.
     * Adjusted based on camera mounting direction (front/back).
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName) * VisionConstants.LIMELIGHT_DIRECTION;
    }

    /**
     * Get vertical offset to target in degrees.
     * Positive means target is above crosshair.
     */
    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Get target area (0-100%).
     */
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * Get the Limelight name for direct LimelightHelpers access.
     */
    public String getName() {
        return limelightName;
    }

    /**
     * Calculate distance to AprilTag in centimeters using vertical angle.
     * Returns 0 if no target is visible.
     */
    public double getDistanceToCM() {
        if (!hasTarget()) {
            return 0.0;
        }

        double ty = getTY();
        double heightDiff = VisionConstants.TAG_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionConstants.CAMERA_ANGLE_DEGREES + ty;
        double distanceMeters = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));

        return distanceMeters * 100.0;  // Convert meters to centimeters
    }

    /**
     * Calculate horizontal offset from center in centimeters.
     * Negative = target is left of center
     * Positive = target is right of center
     * Returns 0 if no target is visible.
     */
    public double getHorizontalOffsetCM() {
        if (!hasTarget()) {
            return 0.0;
        }

        double tx = getTX();  // Horizontal angle in degrees
        double distance = getDistanceToCM() / 100.0;  // Back to meters for calculation

        // Use tan to calculate horizontal offset: offset = distance * tan(angle)
        double offsetMeters = distance * Math.tan(Math.toRadians(tx));

        return offsetMeters * 100.0;  // Convert to centimeters
    }

    /**
     * Get the yaw angle (horizontal angle) to the target.
     * Same as getTX() but with clearer naming for user interface.
     * Negative = target is left of center
     * Positive = target is right of center
     */
    public double getYawAngle() {
        return getTX();
    }

    @Override
    public void periodic() {
        // Read directly from NetworkTables for diagnostics
        double tv_direct = limelightTable.getEntry("tv").getDouble(0.0);
        double tx_direct = limelightTable.getEntry("tx").getDouble(0.0);
        double ty_direct = limelightTable.getEntry("ty").getDouble(0.0);
        double ta_direct = limelightTable.getEntry("ta").getDouble(0.0);
        double tid_direct = limelightTable.getEntry("tid").getDouble(0.0);

        // Calculate telemetry values
        double distanceCM = getDistanceToCM();
        double horizontalOffsetCM = getHorizontalOffsetCM();
        double yawAngle = getYawAngle();

        // Put AprilTag telemetry data on SmartDashboard
        SmartDashboard.putBoolean("LL4/HasTarget", hasTarget());
        SmartDashboard.putNumber("LL4/TagID", getTagID());
        SmartDashboard.putNumber("LL4/Distance_CM", distanceCM);
        SmartDashboard.putNumber("LL4/HorizontalOffset_CM", horizontalOffsetCM);
        SmartDashboard.putNumber("LL4/YawAngle_Deg", yawAngle);
        SmartDashboard.putNumber("LL4/TX_Raw", getTX());  // Debug: direct TX value

        // Log basic vision data
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/TagID", getTagID());
        Logger.recordOutput("Vision/TX", getTX());
        Logger.recordOutput("Vision/TY", getTY());
        Logger.recordOutput("Vision/TA", getTA());
        Logger.recordOutput("Vision/Distance_CM", distanceCM);
        Logger.recordOutput("Vision/HorizontalOffset_CM", horizontalOffsetCM);
        Logger.recordOutput("Vision/YawAngle_Deg", yawAngle);

        // Log direct NetworkTables reads for comparison
        Logger.recordOutput("Vision/Direct/tv", tv_direct);
        Logger.recordOutput("Vision/Direct/tx", tx_direct);
        Logger.recordOutput("Vision/Direct/ty", ty_direct);
        Logger.recordOutput("Vision/Direct/ta", ta_direct);
        Logger.recordOutput("Vision/Direct/tid", tid_direct);

        // Log connection status
        Logger.recordOutput("Vision/TableExists", limelightTable.containsKey("tx"));
        Logger.recordOutput("Vision/LimelightName", limelightName);
    }
}

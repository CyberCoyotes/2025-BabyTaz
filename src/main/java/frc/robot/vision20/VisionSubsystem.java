package frc.robot.vision20;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision20.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    // Access the Limelight's network table (default name is "limelight")
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Returns true if the Limelight has detected a valid target.
     */
    public boolean hasValidTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Returns the horizontal offset from the crosshair to the target (in degrees).
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical offset from the crosshair to the target (in degrees).
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Returns the target area (useful for estimating target distance).
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * If your pipeline outputs an apriltag ID (for example, via a "tid" key), retrieve it.
     * Adjust the key if necessary.
     */
    public int getAprilTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Estimate the distance to the target using the limelight’s mounting angle and the vertical offset.
     * Formula: distance = (targetHeight - limelightHeight) / tan(mountingAngle + ty)
     */
    public double calculateDistance() {
        double angleToTargetRadians = Math.toRadians(VisionConstants.LIMELIGHT_MOUNTING_ANGLE_DEGREES + getVerticalOffset());
        return (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.LIMELIGHT_HEIGHT_METERS) / Math.tan(angleToTargetRadians);
    }

    /**
     * Set the limelight's pipeline.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Set the LED mode for the limelight.
     * @param mode LED mode value (e.g., 3 for force on, 1 for default).
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    @Override
    public void periodic() {
        // Optional: post vision data to SmartDashboard or log telemetry here
    }
}
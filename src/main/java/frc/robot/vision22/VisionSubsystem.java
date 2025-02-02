package frc.robot.vision22;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VisionSubsystem encapsulates all vision-related functionality.
 * It interfaces with the Limelight to retrieve target data.
 */
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        // Connect to the Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Determines if the Limelight has a valid target.
     * @return true if a target is detected.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * @return the horizontal error in degrees.
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Gets the vertical offset from the crosshair to the target.
     * @return the vertical error in degrees.
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the target area (as a percentage of the image).
     * @return the target area.
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * Gets the skew or rotation of the target.
     * @return the target skew in degrees.
     */
    public double getSkew() {
        return limelightTable.getEntry("ts").getDouble(0.0);
    }

    /**
     * Sets the LED mode of the Limelight.
     * Modes may vary (for example, 0 for pipeline default, 1 for force off, 3 for force on).
     * @param mode the LED mode.
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the pipeline on the Limelight.
     * @param pipeline the pipeline number.
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
}
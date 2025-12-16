package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
     */
    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
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

    @Override
    public void periodic() {
        // Read directly from NetworkTables for diagnostics
        double tv_direct = limelightTable.getEntry("tv").getDouble(0.0);
        double tx_direct = limelightTable.getEntry("tx").getDouble(0.0);
        double ty_direct = limelightTable.getEntry("ty").getDouble(0.0);
        double ta_direct = limelightTable.getEntry("ta").getDouble(0.0);
        double tid_direct = limelightTable.getEntry("tid").getDouble(0.0);

        // Log basic vision data
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/TagID", getTagID());
        Logger.recordOutput("Vision/TX", getTX());
        Logger.recordOutput("Vision/TY", getTY());
        Logger.recordOutput("Vision/TA", getTA());

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

package frc.robot.visionV16;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final LEDSubsystem ledSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private String limelightName;

    public VisionSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain, LEDSubsystem ledSubsystem) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        this.ledSubsystem = ledSubsystem;
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // By default, turn off LEDs
        setLeds(false);
    }

    @Override
    public void periodic() {
        // Log vision data to SmartDashboard
        SmartDashboard.putBoolean("Has Target", hasTarget());
        SmartDashboard.putNumber("Target ID", getTagId());
        SmartDashboard.putNumber("TX", getTX());
        SmartDashboard.putNumber("TY", getTY());
    }

    // Basic target detection
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    // Get AprilTag ID (-1 if no tag detected)
    public double getTagId() {
        return limelightTable.getEntry("tid").getDouble(-1);
    }

    // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
    public double getTX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    // Target Area (0% of image to 100% of image)
    public double getTA() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    // Control Limelight LED state
    public void setLeds(boolean enabled) {
        limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
    }

    // Additional helper methods for specific game tasks
    public boolean isAlignedToTarget() {
        return hasTarget() && Math.abs(getTX()) < 2.0; // Within 2 degrees
    }
}
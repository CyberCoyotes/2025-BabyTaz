package frc.robot.visionV17;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers; // Using Limelight's helper class
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem17 extends SubsystemBase {
    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem17(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Update dashboard with vision data
        SmartDashboard.putBoolean("Has Target", hasTarget());
        SmartDashboard.putNumber("Target ID", getTagId());
        SmartDashboard.putNumber("TX", getTX());
        SmartDashboard.putNumber("TY", getTY());
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTagId() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public void setLeds(boolean enabled) {
        // LimelightHelpers.setLEDMode(limelightName, enabled ? 3 : 1);
    }

    // Gets robot pose from Limelight
    public Pose2d getBotPose() {
        double[] botpose = LimelightHelpers.getBotpose(limelightName);
        if (botpose.length > 0) {
            return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        }
        return null;
    }
}
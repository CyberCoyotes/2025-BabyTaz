package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.vision.VisionState;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private static final int APRIL_TAG_PIPELINE = 0;
    private static final int RETRO_PIPELINE = 1;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Start with AprilTag pipeline
        LimelightHelpers.setPipelineIndex("limelight", APRIL_TAG_PIPELINE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    public double getTargetXOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    public double getTargetYOffset() {
        return LimelightHelpers.getTY("limelight");
    }

    public double getTargetArea() {
        return LimelightHelpers.getTA("limelight");
    }
}
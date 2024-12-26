package frc.robot.vision;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private static final int APRIL_TAG_PIPELINE = 0;
    
    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Start with AprilTag pipeline
        LimelightHelpers.setPipelineIndex("limelight", APRIL_TAG_PIPELINE);
        LimelightHelpers.setLEDMode_ForceOff("limelight");

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
package frc.robot.vision19;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.vision19.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    private double lastSeenTagId = -1;
    private double lastValidTimestamp = 0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        configureLimelight();
    }

    private void configureLimelight() {
        // Configure Limelight pipeline for AprilTags
        LimelightHelpers.setPipelineIndex("limelight", 0);
    }

    @Override
    public void periodic() {
        updateVisionData();
        publishTelemetry();
    }

    private void updateVisionData() {
        if (hasValidTarget()) {
            lastSeenTagId = LimelightHelpers.getFiducialID("limelight");
            
            lastValidTimestamp = LimelightHelpers.getLatency_Pipeline("limelight");
            
            // If using vision-assisted odometry:
            if (shouldUpdateOdometry()) {
                var botpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
                drivetrain.addVisionMeasurement(botpose, lastValidTimestamp);
            }
        }
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV("limelight") && 
               LimelightHelpers.getTA("limelight") > VisionConstants.MIN_TARGET_AREA;
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

    public double getTagId() {
        return lastSeenTagId;
    }

    public void setLeds(boolean enabled) {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    private boolean shouldUpdateOdometry() {
        return hasValidTarget(); // && 
            //    LimelightHelpers.getBotPose_TAOnly("limelight") >= 2 &&  // At least 2 tags visible
            //    LimelightHelpers.getPoseAmbiguity("limelight") < VisionConstants.MAX_POSE_AMBIGUITY;

            //    LimelightHelpers.getBotPose_TAOnly("limelight") >= 2 &&  // At least 2 tags visible
            //    LimelightHelpers. getPoseAmbiguity("limelight") < VisionConstants.MAX_POSE_AMBIGUITY
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("V19/HasTarget", hasValidTarget());
        SmartDashboard.putNumber("V19/TagID", lastSeenTagId);
        SmartDashboard.putNumber("V19/TX", getTargetXOffset());
        SmartDashboard.putNumber("V19/TY", getTargetYOffset());
        SmartDashboard.putNumber("V19/TA", getTargetArea());
    }
}
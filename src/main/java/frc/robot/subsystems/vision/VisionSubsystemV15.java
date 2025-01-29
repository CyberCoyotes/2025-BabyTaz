// VisionSubsystem.java
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystemV15 extends SubsystemBase {
    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;
    private Transform3d lastValidTargetPose;
    private double lastValidTimestamp = 0;

    public VisionSubsystemV15(String name, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = name;
        this.drivetrain = drivetrain;
        this.lastValidTargetPose = new Transform3d();
    }

    @Override
    public void periodic() {
        // Update dashboard with vision data
        SmartDashboard.putBoolean("V15/HasTarget", hasTarget());
        SmartDashboard.putNumber("V15/TagID", getTagId());
        SmartDashboard.putNumber("V15/TX", getTX());
        SmartDashboard.putNumber("V15/TY", getTY());
        SmartDashboard.putNumber("V15/TargetArea", getTA());

        if (hasTarget()) {
            lastValidTargetPose = getTargetPose();
            lastValidTimestamp = Timer.getFPGATimestamp() - 
                               LimelightHelpers.getLatency_Pipeline(limelightName)/1000.0;
        }
    }

    public String getLimelightName() { 
        return limelightName; 
    }
    
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTagId() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    public Transform3d getTargetPose() {
        return new Transform3d(
            LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation(),
            LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation()
        );
    }

    public Optional<Transform3d> getLatencyCompensatedPose() {
        if (!hasTarget()) return Optional.empty();
        
        double currentTime = Timer.getFPGATimestamp();
        // Only return poses that are less than 0.1 seconds old
        if (currentTime - lastValidTimestamp > VisionConstantsV15.POSE_HISTORY_SECONDS) {
            return Optional.empty();
        }
        
        return Optional.of(lastValidTargetPose);
    }

    public void setLeds(boolean enabled) {
        if (enabled) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        }
    }

    /**
     * Check if target is valid based on various criteria
     */
    public boolean isTargetValid() {
        if (!hasTarget()) return false;
        
        // Check if target area is too small
        if (getTA() < VisionConstantsV15.MIN_TARGET_AREA) return false;
        
        // Get target pose and check distance
        Transform3d pose = getTargetPose();
        if (pose == null) return false;
        
        double distance = pose.getTranslation().getNorm();
        if (distance > VisionConstantsV15.MAX_VALID_DISTANCE) return false;

        // Check if tx/ty values are reasonable
        double tx = getTX();
        double ty = getTY();
        if (Double.isNaN(tx) || Double.isNaN(ty)) return false;
        
        // Check for valid AprilTag IDs (1-8)
        double tagId = getTagId();
        if (tagId < 1 || tagId > 8) return false;

        return true;
    }
}
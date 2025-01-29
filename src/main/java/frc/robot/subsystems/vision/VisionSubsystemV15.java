package frc.robot.subsystems.vision;

// VisionSubsystem.java
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.VisionConstants;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import java.util.Optional;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionConstantsV15;

public class VisionSubsystemV15 extends SubsystemBase {
private final String limelightName;


    // Use the constant from VisionConstants
/* private final TimeInterpolatableBuffer<Transform3d> poseHistory = 
TimeInterpolatableBuffer.<Transform3d>createBuffer(VisionConstantsV15.POSE_HISTORY_SECONDS);
*/    

    private final TimeInterpolatableBuffer<Transform3d> poseHistory = 
        (TimeInterpolatableBuffer<Transform3d>) TimeInterpolatableBuffer.createBuffer(VisionConstantsV15.POSE_HISTORY_SECONDS);

    private final CommandSwerveDrivetrain drivetrain;  // Add if you need drivetrain reference

    // Constructor updated to optionally take drivetrain
    public VisionSubsystemV15(String name, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = name;
        this.drivetrain = drivetrain;
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
    // Use getLatency_Pipeline() instead of getLatency()
    double timestamp = Timer.getFPGATimestamp() - 
                     LimelightHelpers.getLatency_Pipeline(limelightName)/1000.0;
    poseHistory.addSample(timestamp, getTargetPose());
}
    }

    public String getLimelightName() { return limelightName; }
    
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
        return new Transform3d(LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getTranslation(), 
                               LimelightHelpers.getTargetPose3d_RobotSpace(limelightName).getRotation());
    }

    public Optional<Transform3d> getLatencyCompensatedPose() {
        if (!hasTarget()) return Optional.empty();
        return poseHistory.getSample(Timer.getFPGATimestamp());
    }

    public void setLeds(boolean enabled) {
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
    }
}
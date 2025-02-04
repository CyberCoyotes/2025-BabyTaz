package frc.robot.subsystems.vision18;

/*  
 * Super Nurds + Clockwork blended approach
 * https://claude.ai/chat/1cbe37c3-0e41-4c27-8ab5-ea09a91e8165
 * 
 * Follow up to add customization offsets
 * https://claude.ai/chat/594591dc-e8bc-4674-98cc-cfde24d725e7
*/

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem18 extends SubsystemBase {
    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;
    
    private VisionMeasurement lastMeasurement;
    private boolean visionEnabled = true;
    // private final Alert visionAlert = new Alert("Vision system disabled", AlertType.WARNING);
    private final Alert visionAlert = new Alert("Vision system disabled", AlertType.kWarning);

    public VisionSubsystem18(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        
        // Configure Limelight
        configureLimelight();
    }

    private void configureLimelight() {
        // Set pipeline to 0 (AprilTag)
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        
        // Set camera pose
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            VisionConstants18.ROBOT_TO_CAM.getX(),
            VisionConstants18.ROBOT_TO_CAM.getY(),
            VisionConstants18.ROBOT_TO_CAM.getZ(),
            VisionConstants18.ROBOT_TO_CAM.getRotation().getX(),
            VisionConstants18.ROBOT_TO_CAM.getRotation().getY(),
            VisionConstants18.ROBOT_TO_CAM.getRotation().getZ()
        );
    }

    // TODO Debugging method
    @Override
    public void periodic() {
        if (!visionEnabled) {
            visionAlert.set(true);
            return;
        }
        visionAlert.set(false);

        // Add this debug logging
        logRawValues();

        // Existing code...
        processVisionData();
        updateTelemetry();
    }

    /* ORIGINAL before debugging
    @Override
    public void periodic() {
        if (!visionEnabled) {
            visionAlert.set(true);
            return;
        }
        visionAlert.set(false);

        // Process vision data
        processVisionData();
        
        // Log telemetry
        updateTelemetry();
    }
        */

    private void processVisionData() {
        if (!LimelightHelpers.getTV(limelightName)) {
            return;
        }
    
        // Get basic target info  
        double ta = LimelightHelpers.getTA(limelightName);
        if (ta < VisionConstants18.MIN_TARGET_AREA) {
            return; 
        }
    
        // Get pose data
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        if (botpose.length < 6) {
            return;
        }
    
        // Get target info
        double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        double distance = new Translation3d(targetPose[0], targetPose[1], targetPose[2]).getNorm();
        
        // Get tag information 
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        List<Integer> tagIds = new ArrayList<>();
        
        // Check if there are any fiducial targets
        if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial target : results.targets_Fiducials) {
                tagIds.add((int)target.fiducialID);
            }
        }
    
        // Only proceed if we have valid fiducial data
        if (!tagIds.isEmpty()) {
            // Create measurement
            lastMeasurement = new VisionMeasurement(
                new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5])),
                Timer.getFPGATimestamp(),
                LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0,
                results.targets_Fiducials[0].ta, // Using target area instead of ambiguity
                tagIds.size(),
                distance,
                tagIds
            );
    
            // If measurement is valid, add to pose estimator
            if (lastMeasurement.isValid()) {
                Matrix<N3, N1> stdDevs = calculateStdDevs(lastMeasurement.getAvgDistance());
                drivetrain.addVisionMeasurement(
                    lastMeasurement.getPose(),
                    lastMeasurement.getTimestamp(),
                    stdDevs
                );
            }
        }
    }
    
    private Matrix<N3, N1> calculateStdDevs(double distance) {
        // Calculate standard deviations based on distance
        double xyStdDev = VisionConstants18.XY_STD_DEV_COEFFICIENT * Math.pow(distance, 2.0);
        double thetaStdDev = VisionConstants18.THETA_STD_DEV_COEFFICIENT * Math.pow(distance, 2.0);
        
        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    private void updateTelemetry() {
        // Log basic vision data
        Logger.recordOutput("V18/Enabled", visionEnabled);
        Logger.recordOutput("V18/HasTarget", LimelightHelpers.getTV(limelightName));
        Logger.recordOutput("V18/TX", LimelightHelpers.getTX(limelightName));
        Logger.recordOutput("V18/TY", LimelightHelpers.getTY(limelightName));
        Logger.recordOutput("V18/TA", LimelightHelpers.getTA(limelightName));
        Logger.recordOutput("V18/AvgDistance", lastMeasurement.getAvgDistance());
        Logger.recordOutput("V18/TagIDs", lastMeasurement.getTagIds().toString());

        if (lastMeasurement != null) {
            Logger.recordOutput("V18/LastPose", lastMeasurement.getPose());
            Logger.recordOutput("V18/NumTags", lastMeasurement.getNumTags());
            Logger.recordOutput("V18/Latency", lastMeasurement.getLatency());
            Logger.recordOutput("V18/AvgDistance", lastMeasurement.getAvgDistance());
            Logger.recordOutput("V18/TagIDs", lastMeasurement.getTagIds().toString());
        }
    }

    // Control methods
    public void setEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        LimelightHelpers.setLEDMode_PipelineControl(limelightName); // 0 = use pipeline, 1 = off
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public String getName() {
        return limelightName;
    }

    // TODO
    // Add debug method
    public void logRawValues() {
        // Get values multiple ways to compare
        double tx1 = LimelightHelpers.getTX(limelightName);
        double tx2 = NetworkTableInstance.getDefault()
            .getTable(limelightName)
            .getEntry("tx")
            .getDouble(0.0);
            
        SmartDashboard.putString("V18/LimelightName", limelightName);
        SmartDashboard.putNumber("V18/TX_Helper", tx1);
        SmartDashboard.putNumber("V18/TX_Direct", tx2);
        
        // Log if we can actually see the Limelight
        SmartDashboard.putBoolean("V18/LimelightConnected", 
            NetworkTableInstance.getDefault()
                .getTable(limelightName)
                .containsKey("tx"));
    }


}
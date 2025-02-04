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
    // private final Alert visionAlert = new Alert("Vision system disabled",
    // AlertType.WARNING);
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
                VisionConstants18.ROBOT_TO_CAM.getRotation().getZ());
    }

    // Debugging method
    @Override
    public void periodic() {
        if (!visionEnabled) {
            visionAlert.set(true);
            return;
        }
        visionAlert.set(false);

        // Process vision data
        processVisionData();

        // Log all vision-related data
        logVisionData();

        // Log telemetry
        updateTelemetry();
    }

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
                tagIds.add((int) target.fiducialID);
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
                    tagIds);

            // If measurement is valid, add to pose estimator
            if (lastMeasurement.isValid()) {
                Matrix<N3, N1> stdDevs = calculateStdDevs(lastMeasurement.getAvgDistance());
                drivetrain.addVisionMeasurement(
                        lastMeasurement.getPose(),
                        lastMeasurement.getTimestamp(),
                        stdDevs);
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

        // Get tx from Network Tables to compare

        double txNT = NetworkTableInstance.getDefault()
        .getTable(limelightName)
        .getEntry("tx")
        .getDouble(0.0);

        // Log basic vision data
        SmartDashboard.putBoolean("Vsub/Enabled" , visionEnabled);
        SmartDashboard.putNumber("Vsub/AprilTagID", LimelightHelpers.getFiducialID(limelightName));
        SmartDashboard.putString("Vsub/TagIDs", lastMeasurement.getTagIds().toString());
        SmartDashboard.putNumber("Vsub/TX", LimelightHelpers.getTX(limelightName));
        SmartDashboard.putNumber("Vsub/TX_N.T.", txNT);
        SmartDashboard.putNumber("Vsub/TY", LimelightHelpers.getTY(limelightName));
        SmartDashboard.putNumber("Vsub/TA", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putNumber("Vsub/AvgDistance", lastMeasurement.getAvgDistance());
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

    private void logVisionData() {
        double txNT = NetworkTableInstance.getDefault()
        .getTable(limelightName)
        .getEntry("tx")
        .getDouble(0.0);

        // Basic target detection
        Logger.recordOutput("Vsub/HasTarget", LimelightHelpers.getTV(limelightName));
        Logger.recordOutput("Vsub/AprilTagID", LimelightHelpers.getFiducialID(limelightName));

        // Raw targeting data
        Logger.recordOutput("Vsub/TX", LimelightHelpers.getTX(limelightName));
        Logger.recordOutput("Vsub/TX_NetTables", txNT);
        Logger.recordOutput("Vsub/TY", LimelightHelpers.getTY(limelightName));
        Logger.recordOutput("Vsub/TA", LimelightHelpers.getTA(limelightName));

        // 3D pose data
        Logger.recordOutput("Vsub/BotPose", getCurrentPose());
        Logger.recordOutput("Vsub/TargetPose", getTargetPose());
        Logger.recordOutput("Vsub/TargetDistance", getTargetDistance());

        // System status
        Logger.recordOutput("Vsub/Pipeline", LimelightHelpers.getCurrentPipelineIndex(limelightName));
        Logger.recordOutput("Vsub/Latency", LimelightHelpers.getLatency_Pipeline(limelightName));
        // Logger.recordOutput("Vision/FPS", LimelightHelpers.get (limelightName));
    }

    // Helper method for pose logging
    private Pose2d getCurrentPose() {
        double[] botpose = LimelightHelpers.getBotPose_wpiBlue(limelightName);
        if (botpose.length >= 6) {
            return new Pose2d(
                    botpose[0],
                    botpose[1],
                    Rotation2d.fromDegrees(botpose[5]));
        }
        return new Pose2d();
    }

    // Helper method for target pose
    private Translation3d getTargetPose() {
        double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (targetPose.length >= 3) {
            return new Translation3d(targetPose[0], targetPose[1], targetPose[2]);
        }
        return new Translation3d();
    }

    // Helper method for distance calculation
    private double getTargetDistance() {
        double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        if (targetPose.length >= 3) {
            return new Translation3d(targetPose[0], targetPose[1], targetPose[2]).getNorm();
        }
        return 0.0;
    }

    public void logDiagnostics() {
        // Connection status
        Logger.recordOutput("Vision/Connected", 
            NetworkTableInstance.getDefault()
                .getTable(limelightName)
                .containsKey("tx"));
                
        // Camera settings
        // Logger.recordOutput("Vision/LEDMode", LimelightHelpers.getLED` getLEDMode(limelightName));
        // Logger.recordOutput("Vision/CameraMode", LimelightHelpers.getCameraMode(limelightName));
        
        // Processing stats
        Logger.recordOutput("Vision/PipelineLatency", LimelightHelpers.getLatency_Pipeline(limelightName));
        Logger.recordOutput("Vision/CaptureLatency", LimelightHelpers.getLatency_Capture(limelightName));
        // Logger.recordOutput("Vision/JsonLatency", LimelightHelpers.getLatency_JsonParse(limelightName));
    }

    public void logAlignmentData() {
        // Record alignment errors
        if (lastMeasurement != null) {
            Logger.recordOutput("Vision/AlignX_Error", lastMeasurement.getPose().getX() - VisionConstants18.TARGET_DISTANCE_METERS);
            Logger.recordOutput("Vision/AlignY_Error", lastMeasurement.getPose().getY());
            Logger.recordOutput("Vision/AlignRotation_Error", lastMeasurement.getPose().getRotation().getDegrees());
        }
        
        // Record control outputs
        // Logger.recordOutput("Vision/IsAligning", isAligning());
        // Logger.recordOutput("Vision/AlignmentComplete", isAlignmentComplete());
    }

    public static void setupAdvantagescopeLayout() {
        // Vision tab layout
        Logger.recordMetadata("Vsub/TabName", "Vision Debug");
        
        // Field view
        Logger.recordMetadata("Vsub/BotPose/Position", "Field2d");
        Logger.recordMetadata("Vsub/TargetPose/Position", "Field2d");
        
        // Time series plots
        Logger.recordMetadata("Vsub/TX", "Line Plot");
        Logger.recordMetadata("Vsub/TY", "Line Plot");
        Logger.recordMetadata("Vsub/TargetDistance", "Line Plot");
        
        // Status indicators
        // Logger.recordMetadata("Vsub/HasTarget", "Boolean");
        // Logger.recordMetadata("Vsub/Connected", "Boolean");
        // Logger.recordMetadata("Vsub/IsAligning", "Boolean");
    }
}
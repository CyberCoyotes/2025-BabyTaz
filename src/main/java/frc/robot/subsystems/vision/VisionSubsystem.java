package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers.*;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDSubsystem leds;
    private final AprilTagFieldLayout fieldLayout;
    private final MedianFilter distanceFilter = new MedianFilter(3);
    private final MedianFilter angleFilter = new MedianFilter(3);

    private Pose2d lastRobotPose = new Pose2d();
    private double distanceToTargetMeters = Double.NaN;
    private boolean isTargetValid = false;
    private double lastValidTimestamp = 0.0;
    private int consecutiveValidMeasurements = 0;
    private int consecutiveInvalidMeasurements = 0;
    private int currentPipelineIndex = 0;

    // FIXME
    private boolean lastLEDState = false;
    private int stableCount = 0;
    private static final int REQUIRED_STABLE_CYCLES = 10;


    private static final double MAX_POSE_CHANGE_METERS = 1.0;
    private static final double MAX_VALID_DISTANCE = 5.0;
    private static final double MIN_VALID_DISTANCE = 0.5;
    private static final double MAX_TAG_AGE_SECONDS = 0.25;
    private static final double MIN_TAG_AREA = 0.001;
    private static final double MAX_AMBIGUITY_SCORE = 0.2;
    private static final int REQUIRED_VALID_FRAMES = 3;
    private static final int MAX_INVALID_FRAMES = 5;
    private static final double MIN_CONFIDENCE_THRESHOLD = 0.6;
    private static final double MAX_DATA_AGE_SECONDS = 0.25; // 250ms maximum age for vision data


    private static class LimelightConstants {
        static final double MOUNT_HEIGHT_METERS = 0.5;
        static final double TARGET_HEIGHT_METERS = 1.45;
        static final double MOUNT_ANGLE_RADIANS = Math.toRadians(30.0);
        static final double FORWARD_OFFSET = 0.2;
        static final double SIDE_OFFSET = 0.0;
        static final double HEIGHT_OFFSET = 0.5;
        static final double ROLL_DEGREES = 0.0;
        static final double PITCH_DEGREES = 30.0;
        static final double YAW_DEGREES = 0.0;
    }

    public VisionSubsystem(String limelightName, VisionIO io, CommandSwerveDrivetrain drivetrain,
            LEDSubsystem leds, AprilTagFieldLayout fieldLayout) {
        this.limelightName = limelightName;
        this.io = io;
        this.drivetrain = drivetrain;
        this.leds = leds;
        this.fieldLayout = fieldLayout;

        initializeLimelight();
    }

    private void initializeLimelight() {
        try {
            LimelightHelpers.setPipelineIndex(limelightName, currentPipelineIndex);
            setLEDMode(false);
            configureLimelightPose();
            selectAprilTagPipeline();
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize Limelight: " + e.getMessage(), e.getStackTrace());
        }
    }

    private void configureLimelightPose() {
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                LimelightConstants.FORWARD_OFFSET,
                LimelightConstants.SIDE_OFFSET,
                LimelightConstants.HEIGHT_OFFSET,
                LimelightConstants.ROLL_DEGREES,
                LimelightConstants.PITCH_DEGREES,
                LimelightConstants.YAW_DEGREES);
    }

    private void selectAprilTagPipeline() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            currentPipelineIndex = (alliance.get() == DriverStation.Alliance.Red) ? 1 : 0;
            LimelightHelpers.setPipelineIndex(limelightName, currentPipelineIndex);
        }
    }

    public void periodic() {
        try {
            io.updateInputs(inputs);
            validateData();  // New name to better reflect what we're doing
    
            if (validateTarget()) {
                handleValidTarget();
            } else {
                handleInvalidTarget();
            }
    
            updateLEDs();
            logTelemetry();
        } catch (Exception e) {
            handleSubsystemError(e);
        }
    }
    
    private void validateData() {
        double currentTime = Timer.getFPGATimestamp();
        double dataAge = currentTime - inputs.lastTimeStamp;
        
        if (dataAge > MAX_DATA_AGE_SECONDS) {
            isTargetValid = false;
            return;
        }
    }

    private boolean validateTarget() {
        if (!inputs.hasTargets || inputs.tagId < 0) {
            DataLogManager.log("Target invalid: No targets or invalid tag ID");
            return false;
        }
        if (Timer.getFPGATimestamp() - lastValidTimestamp > MAX_TAG_AGE_SECONDS) {
            DataLogManager.log("Target invalid: Data too old");
            return false;
        }
        if (fieldLayout == null || !fieldLayout.getTags().stream().anyMatch(tag -> tag.ID == inputs.tagId)) {
            DataLogManager.log("Target invalid: Tag not in field layout");
            return false;
        }
    
        try {
            if (!validateAngleAndDistance()) {
                DataLogManager.log("Target invalid: Failed angle/distance validation");
                return false;
            }
            if (!validateFiducial()) {
                DataLogManager.log("Target invalid: Failed fiducial validation");
                return false;
            }
            if (!validatePoseChange()) {
                DataLogManager.log("Target invalid: Failed pose change validation");
                return false;
            }
    
            return true;
        } catch (Exception e) {
            DataLogManager.log("Target validation error: " + e.getMessage());
            return false;
        }
    }
    
    private void handleInvalidTarget() {
        consecutiveValidMeasurements = 0;
        consecutiveInvalidMeasurements++;
        if (consecutiveInvalidMeasurements >= MAX_INVALID_FRAMES) {
            if (isTargetValid) {  // Only log when state actually changes
                isTargetValid = false;
                DataLogManager.log("Target state changed to INVALID after " + 
                    consecutiveInvalidMeasurements + " invalid frames");
            }
        }
    }
    
    private void handleValidTarget() {
        consecutiveValidMeasurements++;
        consecutiveInvalidMeasurements = 0;
    
        if (consecutiveValidMeasurements >= REQUIRED_VALID_FRAMES) {
            if (!isTargetValid) {  // Only log when state actually changes
                isTargetValid = true;
                DataLogManager.log("Target state changed to VALID after " + 
                    consecutiveValidMeasurements + " valid frames");
            }
            var poseEstimate = getPoseEstimate();
            if (poseEstimate != null) {
                updateDrivetrainPose(poseEstimate);
            }
            lastValidTimestamp = inputs.lastTimeStamp;
        }
    }
    
    private void handleSubsystemError(Exception e) {
        DriverStation.reportError("Vision subsystem error: " + e.getMessage(), e.getStackTrace());
        isTargetValid = false;
        consecutiveValidMeasurements = 0;
        consecutiveInvalidMeasurements = MAX_INVALID_FRAMES;
        setLEDMode(false);
    }


    private boolean validateAngleAndDistance() {
        double theta = angleFilter.calculate(
                inputs.verticalAngleRadians + LimelightConstants.MOUNT_ANGLE_RADIANS);
        if (Math.abs(theta) < 0.001)
            return false;

        double heightDelta = LimelightConstants.TARGET_HEIGHT_METERS - LimelightConstants.MOUNT_HEIGHT_METERS;
        distanceToTargetMeters = distanceFilter.calculate(heightDelta / Math.tan(theta));

        return distanceToTargetMeters >= MIN_VALID_DISTANCE &&
                distanceToTargetMeters <= MAX_VALID_DISTANCE;
    }

    private boolean validateFiducial() {
        try {
            // Use public API methods instead of getRawFiducials
            if (!LimelightHelpers.getTV(limelightName))
                return false;

            double area = LimelightHelpers.getTA(limelightName);
            var results = LimelightHelpers.getLatestResults(limelightName);

            return area >= MIN_TAG_AREA &&
                    results.valid &&
                    results.botpose_tagcount >= 1;
        } catch (Exception e) {
            DriverStation.reportError("Failed to validate fiducial: " + e.getMessage(), e.getStackTrace());
            return false;
        }
    }

    private boolean validatePoseChange() {
        if (inputs.botpose.length < 6)
            return false;

        Pose2d currentPose = new Pose2d(
                inputs.botpose[0],
                inputs.botpose[1],
                new Rotation2d(inputs.botpose[5]));

        if (!lastRobotPose.equals(new Pose2d())) {
            Transform2d poseChange = new Transform2d(lastRobotPose, currentPose);
            if (poseChange.getTranslation().getNorm() > MAX_POSE_CHANGE_METERS) {
                return false;
            }
        }
        lastRobotPose = currentPose;
        return true;
    }

    private PoseEstimate getPoseEstimate() {
        try {
            var alliance = DriverStation.getAlliance();
            if (!alliance.isPresent())
                return null;

            PoseEstimate estimate = (alliance.get() == DriverStation.Alliance.Red)
                    ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

            return (estimate != null && !estimate.pose.equals(new Pose2d())) ? estimate : null;
        } catch (Exception e) {
            DriverStation.reportError("Pose estimation error: " + e.getMessage(), e.getStackTrace());
            return null;
        }
    }

    // Modify the updateDrivetrainPose method
    private void updateDrivetrainPose(PoseEstimate estimate) {
        try {
            if (!isTargetValid || estimate == null)
                return;

            double confidence = calculateConfidence(estimate);
            if (confidence >= MIN_CONFIDENCE_THRESHOLD) {
                // Create standard deviations matrix from confidence
                Matrix<N3, N1> stdDevs = new Matrix<>(N3.instance, N1.instance);
                // Lower confidence = higher standard deviation
                double standardDeviation = (1.0 - confidence) * 10; // Scale factor of 10
                stdDevs.set(0, 0, standardDeviation);
                stdDevs.set(1, 0, standardDeviation);
                stdDevs.set(2, 0, standardDeviation * 2); // Higher uncertainty for rotation

                drivetrain.addVisionMeasurement(
                        estimate.pose,
                        estimate.timestampSeconds,
                        stdDevs);
            }
        } catch (Exception e) {
            DriverStation.reportError("Drivetrain update error: " + e.getMessage(), e.getStackTrace());
        }
    }

    private double calculateConfidence(PoseEstimate estimate) {
        double distanceConfidence = 1.0 - (distanceToTargetMeters / MAX_VALID_DISTANCE);
        double latencyConfidence = 1.0 - (estimate.latency / 100.0);
        double areaConfidence = estimate.avgTagArea / 0.1;
        double stabilityConfidence = consecutiveValidMeasurements / (double) REQUIRED_VALID_FRAMES;
        double tagCountConfidence = Math.min(1.0, estimate.tagCount / 3.0);

        return Math.min(1.0,
                Math.min(distanceConfidence,
                        Math.min(latencyConfidence,
                                Math.min(areaConfidence,
                                        Math.min(stabilityConfidence, tagCountConfidence)))));
    }

    // FIXME
    private void updateLEDs() {
    boolean shouldLEDsBeOn = !isTargetValid;
    DataLogManager.log("VisionSubsystem: Target valid: " + isTargetValid + 
                      ", LED state requested: " + shouldLEDsBeOn);
    setLEDMode(shouldLEDsBeOn);
}
    

    public void setLEDMode(boolean on) {
        io.setLeds(on);
        if (leds != null) {
            leds.setVisionLEDState(on);
        }
    }

    public boolean hasValidTarget() {
        return isTargetValid;
    }

    public double getTargetXOffset() {
        return inputs.horizontalAngleRadians;
    }

    public double getTargetArea() {
        return LimelightHelpers.getTA(limelightName);
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", isTargetValid);
        SmartDashboard.putNumber("Vision/Distance", distanceToTargetMeters);
        SmartDashboard.putNumber("Vision/ConsecutiveValid", consecutiveValidMeasurements);
        SmartDashboard.putNumber("Vision/LastValidTime", lastValidTimestamp);
        SmartDashboard.putNumber("Vision/TagID", inputs.tagId);
        SmartDashboard.putNumber("Vision/Pipeline", currentPipelineIndex);

        try {
            PoseEstimate estimate = getPoseEstimate();
            if (estimate != null) {
                SmartDashboard.putNumber("Vision/TagCount", estimate.tagCount);
                SmartDashboard.putNumber("Vision/Confidence", calculateConfidence(estimate));
                SmartDashboard.putNumber("Vision/TimestampDeviation",
                        Math.abs(estimate.timestampSeconds - inputs.lastTimeStamp));
            }
        } catch (Exception e) {
            DriverStation.reportError("Telemetry error: " + e.getMessage(), e.getStackTrace());
        }
    }

    
}
// PoseFusionSubsystem.java
// Purpose: Continuously fuse Limelight 4 vision data into the drivetrain's pose estimator
// Location: src/main/java/frc/robot/subsystems/PoseFusionSubsystem.java

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.Logger;

/**
 * PoseFusionSubsystem - Continuously fuses Limelight 4 vision data with odometry.
 * 
 * === WHAT THIS DOES ===
 * Every robot cycle (50Hz), this subsystem:
 * 1. Gets the latest AprilTag pose estimate from the Limelight
 * 2. Validates the measurement (rejects bad data)
 * 3. Calculates appropriate trust levels (standard deviations)
 * 4. Feeds the measurement to the drivetrain's Kalman filter
 * 
 * === WHY THIS MATTERS ===
 * Wheel odometry drifts over time due to:
 * - Wheel slip
 * - Measurement noise
 * - Tread wear
 * - Floor variations
 * 
 * Vision provides absolute position references (AprilTags are fixed on the field),
 * so we can correct odometry drift continuously.
 * 
 * === THE KEY CONCEPT: STANDARD DEVIATIONS ===
 * The pose estimator uses a Kalman filter that blends two information sources:
 * - Odometry (wheel encoders + gyro)
 * - Vision (Limelight + AprilTags)
 * 
 * Standard deviation tells the filter how much to trust each source:
 * - Lower std dev = higher trust = more weight in the blend
 * - Higher std dev = lower trust = less weight in the blend
 * 
 * Example:
 * - Odometry std dev: 0.05m (we trust it pretty well)
 * - Vision std dev: 0.3m (we trust it less per-measurement)
 * 
 * But vision doesn't drift! So over time, vision corrections keep us accurate.
 * 
 * === MEGATAG1 vs MEGATAG2 ===
 * MegaTag1: Uses multiple AprilTags to calculate robot pose (3D solve)
 *           - More accurate X, Y, AND heading
 *           - Only works when seeing multiple tags
 * 
 * MegaTag2: Uses your robot's gyro + single tag for pose
 *           - Works with single tags
 *           - Uses YOUR heading, calculates X/Y from that
 *           - Requires setRobotOrientation() to be called with gyro data
 * 
 * We primarily use MegaTag2 because:
 * - Works more often (single tag is enough)
 * - Your gyro is probably better than vision heading anyway
 * - More stable estimates
 * 
 * @author CyberCoyotes - Educational Vision Fusion
 */
public class PoseFusionSubsystem extends SubsystemBase {
    
    // ============ DEPENDENCIES ============
    private final CommandSwerveDrivetrain drivetrain;
    private final String limelightName;
    
    // ============ CONFIGURATION ============
    
    /**
     * Enable/disable fusion for A/B testing.
     * Set to false to see how the robot performs with odometry only.
     */
    private boolean fusionEnabled = true;
    
    /**
     * Standard deviations for multi-tag estimates (MegaTag1 or multiple tags in view).
     * These are MORE trusted because multiple tags give redundant information.
     * 
     * Format: [x meters, y meters, theta radians]
     */
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = 
        VecBuilder.fill(0.3, 0.3, 0.5);
    
    /**
     * Standard deviations for single-tag MegaTag2 estimates.
     * We trust X and Y moderately, but NOT the heading (using gyro instead).
     * 
     * The huge number for theta means "ignore vision heading completely."
     */
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = 
        VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    
    /**
     * Maximum angular velocity (deg/s) before rejecting vision updates.
     * When spinning fast, the image is blurry and pose estimates are unreliable.
     */
    private static final double MAX_ANGULAR_VELOCITY_DPS = 720.0;
    
    /**
     * Minimum tag area (percentage of image) to accept single-tag updates.
     * Small tags = far away = less accurate.
     */
    private static final double MIN_TAG_AREA = 0.08;  // 8% of image
    
    /**
     * Maximum pose jump (meters) to accept.
     * If vision says we teleported 2 meters, something is wrong.
     */
    private static final double MAX_POSE_JUMP = 1.0;
    
    // ============ STATE TRACKING ============
    
    private PoseEstimate lastAcceptedEstimate = null;
    private int acceptedMeasurements = 0;
    private int rejectedMeasurements = 0;
    private String lastRejectReason = "None";
    
    // For tracking improvement over time
    private Pose2d poseAtLastVisionUpdate = null;
    private double cumulativeVisionCorrection = 0.0;
    
    /**
     * Creates a new PoseFusionSubsystem.
     * 
     * @param drivetrain The swerve drivetrain (provides pose estimator)
     * @param limelightName Network name of the Limelight (e.g., "limelight")
     */
    public PoseFusionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        this.drivetrain = drivetrain;
        this.limelightName = limelightName;
        
        // Log initial configuration
        Logger.recordOutput("PoseFusion/Config/LimelightName", limelightName);
        Logger.recordOutput("PoseFusion/Config/MaxAngularVelocity", MAX_ANGULAR_VELOCITY_DPS);
        Logger.recordOutput("PoseFusion/Config/MinTagArea", MIN_TAG_AREA);
        Logger.recordOutput("PoseFusion/Config/MaxPoseJump", MAX_POSE_JUMP);
    }
    
    /**
     * Enable or disable vision fusion.
     * Useful for testing odometry-only vs fusion performance.
     */
    public void setFusionEnabled(boolean enabled) {
        this.fusionEnabled = enabled;
        Logger.recordOutput("PoseFusion/Enabled", enabled);
        System.out.println("Vision fusion " + (enabled ? "ENABLED" : "DISABLED"));
    }
    
    public boolean isFusionEnabled() {
        return fusionEnabled;
    }
    
    /**
     * Reset statistics counters (useful when starting a new test).
     */
    public void resetStatistics() {
        acceptedMeasurements = 0;
        rejectedMeasurements = 0;
        cumulativeVisionCorrection = 0.0;
        lastRejectReason = "None";
        System.out.println("Pose fusion statistics reset");
    }
    
    @Override
    public void periodic() {
        // Always log, even if fusion is disabled (for comparison)
        logCurrentState();
        
        if (!fusionEnabled) {
            Logger.recordOutput("PoseFusion/Status", "DISABLED");
            return;
        }
        
        // Step 1: Update Limelight with our current orientation
        // This is REQUIRED for MegaTag2 to work correctly
        updateLimelightOrientation();
        
        // Step 2: Get pose estimate from Limelight
        PoseEstimate estimate = LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
        // Step 3: Validate the measurement
        RejectReason rejectReason = shouldRejectMeasurement(estimate);
        
        if (rejectReason != RejectReason.NONE) {
            // Log why we rejected it (helpful for debugging)
            rejectedMeasurements++;
            lastRejectReason = rejectReason.name();
            Logger.recordOutput("PoseFusion/Status", "REJECTED: " + rejectReason.name());
            Logger.recordOutput("PoseFusion/LastRejectReason", rejectReason.name());
            return;
        }
        
        // Step 4: Calculate appropriate standard deviations
        Matrix<N3, N1> stdDevs = calculateStandardDeviations(estimate);
        
        // Step 5: Track the correction we're about to apply
        Pose2d currentPose = drivetrain.getState().Pose;
        double correctionMagnitude = currentPose.getTranslation()
            .getDistance(estimate.pose.getTranslation());
        cumulativeVisionCorrection += correctionMagnitude;
        
        // Step 6: Apply the vision measurement to the pose estimator!
        drivetrain.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds,
            stdDevs
        );
        
        // Step 7: Log success and update statistics
        acceptedMeasurements++;
        lastAcceptedEstimate = estimate;
        poseAtLastVisionUpdate = currentPose;
        
        logAcceptedMeasurement(estimate, stdDevs, correctionMagnitude);
    }
    
    /**
     * Updates the Limelight with our current robot orientation.
     * This is CRITICAL for MegaTag2 to work correctly.
     * 
     * MegaTag2 uses your gyro heading to constrain the pose solve,
     * which makes it more stable and accurate.
     */
    private void updateLimelightOrientation() {
        Rotation2d heading = drivetrain.getState().Pose.getRotation();
        
        // Get angular velocity if available (optional but helps)
        double yawRate = 0.0;
        try {
            yawRate = drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        } catch (Exception e) {
            // If we can't get it, just use 0
        }
        
        // Send to Limelight
        // Parameters: heading (deg), pitch rate, pitch, roll rate, roll, yaw rate
        LimelightHelpers.SetRobotOrientation(
            limelightName,
            heading.getDegrees(),
            0, 0, 0, 0,  // We don't use pitch/roll rates
            yawRate
        );
        
        Logger.recordOutput("PoseFusion/SentHeading", heading.getDegrees());
    }
    
    /**
     * Rejection reasons - helpful for understanding why measurements are thrown out.
     */
    private enum RejectReason {
        NONE,               // Not rejected - measurement is good!
        NULL_ESTIMATE,      // Limelight returned null
        NO_TAGS,           // No AprilTags visible
        SPINNING_TOO_FAST, // Robot rotating too quickly for accurate vision
        TAG_TOO_SMALL,     // Tag is too far away / too small in frame
        POSE_JUMP,         // Measurement is too different from current pose
        INVALID_TIMESTAMP  // Timestamp is in the future or way in the past
    }
    
    /**
     * Validates a pose estimate and returns why it should be rejected (if at all).
     * 
     * @param estimate The pose estimate from Limelight
     * @return RejectReason.NONE if valid, otherwise the reason for rejection
     */
    private RejectReason shouldRejectMeasurement(PoseEstimate estimate) {
        // Check for null
        if (estimate == null) {
            return RejectReason.NULL_ESTIMATE;
        }
        
        // Check for no tags
        if (estimate.tagCount == 0) {
            return RejectReason.NO_TAGS;
        }
        
        // Check angular velocity
        double angularVelocity = 0.0;
        try {
            angularVelocity = Math.abs(
                drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        } catch (Exception e) {
            // If we can't read it, don't reject for this reason
        }
        
        if (angularVelocity > MAX_ANGULAR_VELOCITY_DPS) {
            return RejectReason.SPINNING_TOO_FAST;
        }
        
        // Check tag size for single-tag updates
        if (estimate.tagCount == 1) {
            double tagArea = LimelightHelpers.getTA(limelightName);
            if (tagArea < MIN_TAG_AREA) {
                return RejectReason.TAG_TOO_SMALL;
            }
        }
        
        // Check for pose jumps (sanity check)
        Pose2d currentPose = drivetrain.getState().Pose;
        double poseDistance = currentPose.getTranslation()
            .getDistance(estimate.pose.getTranslation());
        
        if (poseDistance > MAX_POSE_JUMP) {
            return RejectReason.POSE_JUMP;
        }
        
        // Check timestamp validity
        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (estimate.timestampSeconds > currentTime + 0.1 ||
            estimate.timestampSeconds < currentTime - 1.0) {
            return RejectReason.INVALID_TIMESTAMP;
        }
        
        // All checks passed!
        return RejectReason.NONE;
    }
    
    /**
     * Calculates how much to trust this measurement.
     * 
     * More tags = more trust (lower std devs)
     * Closer tags = more trust (lower std devs)
     * 
     * @param estimate The pose estimate
     * @return Standard deviations as [x, y, theta] matrix
     */
    private Matrix<N3, N1> calculateStandardDeviations(PoseEstimate estimate) {
        if (estimate.tagCount >= 2) {
            // Multiple tags - use higher trust values
            return MULTI_TAG_STD_DEVS;
        }
        
        // Single tag - scale trust based on distance
        // Farther away = less accurate = higher std devs
        double distanceFactor = Math.max(1.0, estimate.avgTagDist / 3.0);
        
        return VecBuilder.fill(
            0.5 * distanceFactor,   // X uncertainty grows with distance
            0.5 * distanceFactor,   // Y uncertainty grows with distance
            Double.MAX_VALUE        // Never trust single-tag heading
        );
    }
    
    /**
     * Logs the current state for analysis in AdvantageScope.
     */
    private void logCurrentState() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Current pose
        Logger.recordOutput("PoseFusion/CurrentPose", currentPose);
        
        // Statistics
        Logger.recordOutput("PoseFusion/AcceptedCount", acceptedMeasurements);
        Logger.recordOutput("PoseFusion/RejectedCount", rejectedMeasurements);
        Logger.recordOutput("PoseFusion/AcceptRate", 
            acceptedMeasurements + rejectedMeasurements > 0 ?
            (double) acceptedMeasurements / (acceptedMeasurements + rejectedMeasurements) : 0);
        Logger.recordOutput("PoseFusion/CumulativeCorrection", cumulativeVisionCorrection);
        
        // Limelight raw data
        // Logger.recordOutput("PoseFusion/LL/TagCount", 
            // (int) LimelightHelpers.getTV(limelightName));
        Logger.recordOutput("PoseFusion/LL/TagArea", 
            LimelightHelpers.getTA(limelightName));
        
        // SmartDashboard for driver station
        SmartDashboard.putBoolean("Vision Fusion Active", fusionEnabled);
        SmartDashboard.putNumber("Vision Corrections", acceptedMeasurements);
        SmartDashboard.putString("Last Vision Reject", lastRejectReason);
    }
    
    /**
     * Logs details when a measurement is accepted.
     */
    private void logAcceptedMeasurement(PoseEstimate estimate, 
                                         Matrix<N3, N1> stdDevs, 
                                         double correctionMagnitude) {
        Logger.recordOutput("PoseFusion/Status", "ACCEPTED");
        Logger.recordOutput("PoseFusion/LastAccepted/Pose", estimate.pose);
        Logger.recordOutput("PoseFusion/LastAccepted/TagCount", estimate.tagCount);
        Logger.recordOutput("PoseFusion/LastAccepted/AvgTagDist", estimate.avgTagDist);
        Logger.recordOutput("PoseFusion/LastAccepted/Latency", estimate.latency);
        Logger.recordOutput("PoseFusion/LastAccepted/StdDevX", stdDevs.get(0, 0));
        Logger.recordOutput("PoseFusion/LastAccepted/StdDevY", stdDevs.get(1, 0));
        Logger.recordOutput("PoseFusion/LastAccepted/CorrectionMagnitude", correctionMagnitude);
        
        // Log individual tag data if available
        if (estimate.rawFiducials != null) {
            for (int i = 0; i < estimate.rawFiducials.length; i++) {
                var tag = estimate.rawFiducials[i];
                Logger.recordOutput("PoseFusion/Tags/Tag" + i + "/ID", tag.id);
                Logger.recordOutput("PoseFusion/Tags/Tag" + i + "/Distance", tag.distToRobot);
                Logger.recordOutput("PoseFusion/Tags/Tag" + i + "/Ambiguity", tag.ambiguity);
            }
        }
    }
    
    // ============ PUBLIC ACCESSORS ============
    
    public int getAcceptedMeasurements() {
        return acceptedMeasurements;
    }
    
    public int getRejectedMeasurements() {
        return rejectedMeasurements;
    }
    
    public double getCumulativeCorrection() {
        return cumulativeVisionCorrection;
    }
    
    public String getLastRejectReason() {
        return lastRejectReason;
    }
}

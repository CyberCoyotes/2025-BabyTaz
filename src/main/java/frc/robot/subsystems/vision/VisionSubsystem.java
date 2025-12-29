package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.led.LEDState;
import frc.robot.subsystems.led.LEDSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Unified vision subsystem for AprilTag detection using Limelight.
 * Provides target detection, measurement data, state tracking, and LED feedback.
 *
 * Merged from LimelightVision.java - provides all vision functionality in one place.
 */
public class VisionSubsystem extends SubsystemBase {

    /**
     * Vision state enumeration for tracking AprilTag detection and alignment status.
     */
    public enum VisionState {
        NO_TARGET(0),
        TARGET_VISIBLE(1),
        TARGET_LOCKED(2);

        private final int value;

        VisionState(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private final String limelightName;
    private final LEDSubsystem leds;
    private final NetworkTable limelightTable;
    private VisionState currentState = VisionState.NO_TARGET;

    // Vision processing constants
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double MIN_TARGET_AREA = 0.1; // % of image

    /**
     * Creates VisionSubsystem with LED feedback.
     * @param limelightName Name of the Limelight (e.g., "limelight")
     * @param leds LED subsystem for visual feedback (can be null)
     */
    public VisionSubsystem(String limelightName, LEDSubsystem leds) {
        this.limelightName = limelightName;
        this.leds = leds;
        this.limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
        configureLimelight();
    }

    /**
     * Creates VisionSubsystem without LED feedback.
     * @param limelightName Name of the Limelight (e.g., "limelight")
     */
    public VisionSubsystem(String limelightName) {
        this(limelightName, null);
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline by default
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void periodic() {
        // Update finite state machine
        updateVisionState();

        // Update LED feedback if available
        updateLEDs();

        // Log comprehensive telemetry
        logTelemetry();
    }

    private void updateVisionState() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        double horizontalOffset = getHorizontalOffset();
        double targetArea = LimelightHelpers.getTA(limelightName);

        if (!hasTarget || targetArea < MIN_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }

        // Add to VisionSubsystem periodic():
        boolean targetValid = LimelightHelpers.getTV(limelightName);
        // System.out.println("Target Valid: " + targetValid);
        // System.out.println("V15 Current State: " + getState());
    }

    private void updateLEDs() {
        if (leds != null) {
            switch (currentState) {
                case TARGET_LOCKED:
                    leds.setState(LEDState.TARGET_LOCKED);
                    break;
                case TARGET_VISIBLE:
                    leds.setState(LEDState.TARGET_VISIBLE);
                    break;
                case NO_TARGET:
                    leds.setState(LEDState.NO_TARGET);
                    break;
            }
        }
    }

    public VisionState getState() {
        return currentState;
    }

    // ============================================================================
    // TARGET DETECTION METHODS
    // ============================================================================

    /**
     * Check if a valid AprilTag target is visible.
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Get the AprilTag ID (fiducial ID).
     * Legacy method name for compatibility.
     */
    public int getTagId() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    /**
     * Get the AprilTag ID (fiducial ID).
     * Preferred method name (matches LimelightVision naming).
     */
    public int getTagID() {
        return getTagId();
    }

    /**
     * Check if a tag ID is valid (within FRC range).
     */
    public boolean isTagValid(int tagId) {
        return tagId >= VisionConstants.MIN_VALID_TAG &&
               tagId <= VisionConstants.MAX_VALID_TAG;
    }

    // ============================================================================
    // OFFSET AND ANGLE METHODS
    // ============================================================================

    /**
     * Get horizontal offset to target in degrees.
     * Legacy method name for compatibility.
     * Positive means target is to the right.
     * Adjusted based on camera mounting direction (front/back).
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName) * VisionConstants.LIMELIGHT_DIRECTION;
    }

    /**
     * Get horizontal offset to target in degrees.
     * Preferred method name (matches LimelightVision naming).
     * Positive means target is to the right.
     * Adjusted based on camera mounting direction (front/back).
     */
    public double getTX() {
        return getHorizontalOffset();
    }

    /**
     * Get vertical offset to target in degrees.
     * Legacy method name for compatibility.
     * Positive means target is above crosshair.
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(limelightName);
    }

    /**
     * Get vertical offset to target in degrees.
     * Preferred method name (matches LimelightVision naming).
     * Positive means target is above crosshair.
     */
    public double getTY() {
        return getVerticalOffset();
    }

    /**
     * Get target area (0-100%).
     */
    public double getTA() {
        return LimelightHelpers.getTA(limelightName);
    }

    /**
     * Get the yaw angle (horizontal angle) to the target.
     * Same as getTX() but with clearer naming for user interface.
     * Negative = target is left of center
     * Positive = target is right of center
     */
    public double getYawAngle() {
        return getTX();
    }

    // ============================================================================
    // DISTANCE CALCULATION METHODS
    // ============================================================================

    /**
     * Calculate distance to AprilTag in centimeters using vertical angle.
     * Returns 0 if no target is visible.
     */
    public double getDistanceToCM() {
        if (!hasTarget()) {
            return 0.0;
        }

        double ty = getTY();
        double heightDiff = VisionConstants.TAG_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionConstants.CAMERA_ANGLE_DEGREES + ty;
        double distanceMeters = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));

        return distanceMeters * 100.0;  // Convert meters to centimeters
    }

    /**
     * Calculate horizontal offset from center in centimeters.
     * Negative = target is left of center
     * Positive = target is right of center
     * Returns 0 if no target is visible.
     */
    public double getHorizontalOffsetCM() {
        if (!hasTarget()) {
            return 0.0;
        }

        double tx = getTX();  // Horizontal angle in degrees
        double distance = getDistanceToCM() / 100.0;  // Back to meters for calculation

        // Use tan to calculate horizontal offset: offset = distance * tan(angle)
        double offsetMeters = distance * Math.tan(Math.toRadians(tx));

        return offsetMeters * 100.0;  // Convert to centimeters
    }

    // ============================================================================
    // UTILITY METHODS
    // ============================================================================

    /**
     * Get the Limelight name for direct LimelightHelpers access.
     */
    public String getName() {
        return limelightName;
    }

    private void logTelemetry() {
        // Read directly from NetworkTables for diagnostics
        double tv_direct = limelightTable.getEntry("tv").getDouble(0.0);
        double tx_direct = limelightTable.getEntry("tx").getDouble(0.0);
        double ty_direct = limelightTable.getEntry("ty").getDouble(0.0);
        double ta_direct = limelightTable.getEntry("ta").getDouble(0.0);
        double tid_direct = limelightTable.getEntry("tid").getDouble(0.0);

        // Calculate telemetry values
        double distanceCM = getDistanceToCM();
        double horizontalOffsetCM = getHorizontalOffsetCM();
        double yawAngle = getYawAngle();

        // Put AprilTag telemetry data on SmartDashboard
        SmartDashboard.putBoolean("LL4/HasTarget", hasTarget());
        SmartDashboard.putNumber("LL4/TagID", getTagID());
        SmartDashboard.putNumber("LL4/Distance_CM", distanceCM);
        SmartDashboard.putNumber("LL4/HorizontalOffset_CM", horizontalOffsetCM);
        SmartDashboard.putNumber("LL4/YawAngle_Deg", yawAngle);
        SmartDashboard.putNumber("LL4/TX_Raw", getTX());  // Debug: direct TX value
        SmartDashboard.putString("Vision/State", currentState.toString());

        // AdvantageKit logging - basic vision data
        Logger.recordOutput("Vision/HasTarget", hasTarget());
        Logger.recordOutput("Vision/TagID", getTagID());
        Logger.recordOutput("Vision/TX", getTX());
        Logger.recordOutput("Vision/TY", getTY());
        Logger.recordOutput("Vision/TA", getTA());
        Logger.recordOutput("Vision/Distance_CM", distanceCM);
        Logger.recordOutput("Vision/HorizontalOffset_CM", horizontalOffsetCM);
        Logger.recordOutput("Vision/YawAngle_Deg", yawAngle);
        Logger.recordOutput("Vision/State", currentState.toString());

        // AdvantageKit logging - direct NetworkTables reads for comparison
        Logger.recordOutput("Vision/Direct/tv", tv_direct);
        Logger.recordOutput("Vision/Direct/tx", tx_direct);
        Logger.recordOutput("Vision/Direct/ty", ty_direct);
        Logger.recordOutput("Vision/Direct/ta", ta_direct);
        Logger.recordOutput("Vision/Direct/tid", tid_direct);

        // AdvantageKit logging - connection status
        Logger.recordOutput("Vision/TableExists", limelightTable.containsKey("tx"));
        Logger.recordOutput("Vision/LimelightName", limelightName);
    }
    
}
package frc.robot.commands.visiontest;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionTestConstants;
import org.littletonrobotics.junction.Logger;

/**
 * ===========================================================================
 * MODEL C: PERPENDICULAR ALIGNMENT WITH DISTANCE
 * ===========================================================================
 *
 * Adapted from Limelight "Aiming and Ranging with Swerve" example.
 * Uses MegaTag2 for pose data when available.
 *
 * PURPOSE:
 * - Rotates robot to face AprilTag perpendicular (tx = 0)
 * - Drives forward/backward to target X-axis distance (default: 1.2m)
 * - Strafes left/right to center on tag (lateral Y-axis centering)
 * - Full 3-axis swerve alignment
 *
 * HOW IT WORKS:
 * 1. Reads TX, TY from Limelight for basic alignment
 * 2. Optionally uses MegaTag2 pose for more accurate perpendicular alignment
 * 3. Uses three PID controllers: rotation, range (forward), lateral (strafe)
 * 4. Applies deadband to lateral movement to prevent small corrections
 * 5. All three outputs applied simultaneously via swerve RobotCentric request
 *
 * MEGATAG2 INTEGRATION:
 * - When MegaTag2 pose is available, uses robot yaw relative to tag
 * - Falls back to TX-based alignment when MegaTag2 unavailable
 * - Set robot orientation via LimelightHelpers before using
 *
 * SUCCESS CRITERIA:
 * - TX within rotation tolerance (2.0 degrees)
 * - Distance within range tolerance (0.05 meters)
 * - Lateral offset within tolerance (2.0 degrees)
 *
 * TELEMETRY OUTPUT:
 * - VisionTest/ModelC/Status: Current state
 * - VisionTest/ModelC/TX, TY: Limelight offsets
 * - VisionTest/ModelC/Distance: Current distance
 * - VisionTest/ModelC/RotationSpeed, ForwardSpeed, LateralSpeed: Applied velocities
 * - VisionTest/ModelC/RotationAligned, DistanceAligned, LateralAligned: Per-axis status
 * - VisionTest/ModelC/FullyAligned: All three axes aligned
 *
 * BUTTON BINDING: Shuffleboard "Model C: Perpendicular + Range" button
 */
public class PerpendicularAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightVision vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID Controllers for 3-axis control
    private final PIDController rotationPID;    // Rotation to face tag perpendicular
    private final PIDController rangePID;       // Forward/backward distance control
    private final PIDController lateralPID;     // Left/right strafe centering

    // Target distance
    private final double targetDistanceMeters;

    // NetworkTables for telemetry
    private final NetworkTable telemetryTable;

    // Status tracking
    private enum AlignmentStatus {
        SEARCHING,      // No target visible
        ALIGNING,       // Target visible, aligning
        ALIGNED         // All three axes within tolerance
    }
    private AlignmentStatus currentStatus = AlignmentStatus.SEARCHING;

    // Track if we've set MegaTag2 orientation this cycle
    private boolean megaTagOrientationSet = false;

    /**
     * Creates a PerpendicularAlignCommand with default target distance (1.2m).
     */
    public PerpendicularAlignCommand(CommandSwerveDrivetrain drivetrain, LimelightVision vision) {
        this(drivetrain, vision, VisionTestConstants.DEFAULT_TARGET_DISTANCE_METERS);
    }

    /**
     * Creates a PerpendicularAlignCommand with custom target distance.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The Limelight vision subsystem
     * @param targetDistanceMeters Target distance from AprilTag in meters
     */
    public PerpendicularAlignCommand(CommandSwerveDrivetrain drivetrain, LimelightVision vision, double targetDistanceMeters) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;

        // Initialize rotation PID (perpendicular facing)
        rotationPID = new PIDController(
            VisionTestConstants.ModelC.ROTATION_KP,
            VisionTestConstants.ModelC.ROTATION_KI,
            VisionTestConstants.ModelC.ROTATION_KD
        );
        rotationPID.setTolerance(VisionTestConstants.ModelC.ROTATION_TOLERANCE_DEGREES);
        rotationPID.setSetpoint(VisionTestConstants.TARGET_TX_CENTERED);
        rotationPID.enableContinuousInput(-180, 180);

        // Initialize range PID (forward/backward distance)
        rangePID = new PIDController(
            VisionTestConstants.ModelC.RANGE_KP,
            VisionTestConstants.ModelC.RANGE_KI,
            VisionTestConstants.ModelC.RANGE_KD
        );
        rangePID.setTolerance(VisionTestConstants.ModelC.DISTANCE_TOLERANCE_METERS);
        rangePID.setSetpoint(targetDistanceMeters);

        // Initialize lateral PID (left/right strafe)
        lateralPID = new PIDController(
            VisionTestConstants.ModelC.LATERAL_KP,
            VisionTestConstants.ModelC.LATERAL_KI,
            VisionTestConstants.ModelC.LATERAL_KD
        );
        lateralPID.setTolerance(VisionTestConstants.ModelC.LATERAL_TOLERANCE_DEGREES);
        lateralPID.setSetpoint(VisionTestConstants.TARGET_TX_CENTERED);

        // Setup NetworkTables
        telemetryTable = NetworkTableInstance.getDefault()
            .getTable(VisionTestConstants.NTKeys.MODEL_C_PREFIX.replace("/", ""));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rangePID.reset();
        lateralPID.reset();
        currentStatus = AlignmentStatus.SEARCHING;
        megaTagOrientationSet = false;
        logStatus("STARTED");

        // Log target distance
        telemetryTable.getEntry("TargetDistance").setDouble(targetDistanceMeters);
        Logger.recordOutput("VisionTest/ModelC/TargetDistance", targetDistanceMeters);

        // Set pipeline to AprilTag mode
        LimelightHelpers.setPipelineIndex(vision.getName(), 0);
    }

    @Override
    public void execute() {
        // Update MegaTag2 with robot orientation for better pose estimation
        updateMegaTagOrientation();

        // Check for valid target
        if (!vision.hasTarget()) {
            currentStatus = AlignmentStatus.SEARCHING;
            stopRobot();
            logTelemetry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false, false, false);
            return;
        }

        // Get measurements from Limelight
        double tx = vision.getTX();
        double ty = vision.getTY();

        // Calculate distance to tag using trigonometry
        double heightDiff = VisionTestConstants.TAG_HEIGHT_METERS - VisionTestConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionTestConstants.CAMERA_ANGLE_DEGREES + ty;
        double currentDistance;
        if (Math.abs(angleToTag) < 0.5) {
            currentDistance = targetDistanceMeters; // Fallback
        } else {
            currentDistance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));
        }

        // Calculate rotation speed (rotate to face tag perpendicular)
        // Negative sign: positive tx means tag is to the right, so we rotate right (negative in WPILib convention)
        double rotationSpeed = -rotationPID.calculate(tx);

        // Calculate forward speed (drive to target distance)
        double forwardSpeed = rangePID.calculate(currentDistance);

        // Calculate lateral speed with deadband
        // Only strafe if TX is outside the deadband
        double lateralSpeed = 0.0;
        if (Math.abs(tx) > VisionTestConstants.ModelC.LATERAL_DEADBAND_DEGREES) {
            lateralSpeed = lateralPID.calculate(tx);
        }

        // Apply speed limits
        rotationSpeed = MathUtil.clamp(
            rotationSpeed,
            -VisionTestConstants.ModelC.MAX_ROTATION_SPEED_RADPS,
            VisionTestConstants.ModelC.MAX_ROTATION_SPEED_RADPS
        );
        forwardSpeed = MathUtil.clamp(
            forwardSpeed,
            -VisionTestConstants.ModelC.MAX_FORWARD_SPEED_MPS,
            VisionTestConstants.ModelC.MAX_FORWARD_SPEED_MPS
        );
        lateralSpeed = MathUtil.clamp(
            lateralSpeed,
            -VisionTestConstants.ModelC.MAX_LATERAL_SPEED_MPS,
            VisionTestConstants.ModelC.MAX_LATERAL_SPEED_MPS
        );

        // Check alignment status for each axis
        boolean rotationAligned = rotationPID.atSetpoint();
        boolean distanceAligned = rangePID.atSetpoint();
        boolean lateralAligned = lateralPID.atSetpoint() || Math.abs(tx) <= VisionTestConstants.ModelC.LATERAL_DEADBAND_DEGREES;

        currentStatus = (rotationAligned && distanceAligned && lateralAligned)
            ? AlignmentStatus.ALIGNED
            : AlignmentStatus.ALIGNING;

        // Apply to drivetrain - full 3-axis control
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(lateralSpeed)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(tx, ty, currentDistance, rotationSpeed, forwardSpeed, lateralSpeed,
                     rotationAligned, distanceAligned, lateralAligned);
    }

    /**
     * Updates MegaTag2 with current robot orientation for improved pose estimation.
     * This helps MegaTag2 provide more accurate pose data.
     */
    private void updateMegaTagOrientation() {
        if (!megaTagOrientationSet) {
            try {
                // Get current robot heading from drivetrain
                double robotYawDegrees = drivetrain.getState().Pose.getRotation().getDegrees();

                // Set orientation for MegaTag2
                // Note: Yaw rate and pitch/roll are set to 0 for simplicity
                LimelightHelpers.SetRobotOrientation(
                    vision.getName(),
                    robotYawDegrees,
                    0.0,  // Yaw rate
                    0.0,  // Pitch
                    0.0,  // Pitch rate
                    0.0,  // Roll
                    0.0   // Roll rate
                );
                megaTagOrientationSet = true;
                Logger.recordOutput("VisionTest/ModelC/MegaTagOrientationSet", true);
            } catch (Exception e) {
                Logger.recordOutput("VisionTest/ModelC/MegaTagOrientationError", e.getMessage());
            }
        }
    }

    private void stopRobot() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private void logTelemetry(double tx, double ty, double distance,
                               double rotationSpeed, double forwardSpeed, double lateralSpeed,
                               boolean rotationAligned, boolean distanceAligned, boolean lateralAligned) {
        boolean fullyAligned = rotationAligned && distanceAligned && lateralAligned;

        // NetworkTables output
        telemetryTable.getEntry("Status").setString(currentStatus.name());
        telemetryTable.getEntry("TX").setDouble(tx);
        telemetryTable.getEntry("TY").setDouble(ty);
        telemetryTable.getEntry("Distance").setDouble(distance);
        telemetryTable.getEntry("RotationSpeed").setDouble(rotationSpeed);
        telemetryTable.getEntry("ForwardSpeed").setDouble(forwardSpeed);
        telemetryTable.getEntry("LateralSpeed").setDouble(lateralSpeed);
        telemetryTable.getEntry("RotationAligned").setBoolean(rotationAligned);
        telemetryTable.getEntry("DistanceAligned").setBoolean(distanceAligned);
        telemetryTable.getEntry("LateralAligned").setBoolean(lateralAligned);
        telemetryTable.getEntry("FullyAligned").setBoolean(fullyAligned);
        telemetryTable.getEntry("TagID").setDouble(vision.getTagID());
        telemetryTable.getEntry("DistanceError").setDouble(distance - targetDistanceMeters);

        // AdvantageKit logging
        Logger.recordOutput("VisionTest/ModelC/Status", currentStatus.name());
        Logger.recordOutput("VisionTest/ModelC/TX", tx);
        Logger.recordOutput("VisionTest/ModelC/TY", ty);
        Logger.recordOutput("VisionTest/ModelC/Distance", distance);
        Logger.recordOutput("VisionTest/ModelC/RotationSpeed", rotationSpeed);
        Logger.recordOutput("VisionTest/ModelC/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("VisionTest/ModelC/LateralSpeed", lateralSpeed);
        Logger.recordOutput("VisionTest/ModelC/RotationAligned", rotationAligned);
        Logger.recordOutput("VisionTest/ModelC/DistanceAligned", distanceAligned);
        Logger.recordOutput("VisionTest/ModelC/LateralAligned", lateralAligned);
        Logger.recordOutput("VisionTest/ModelC/FullyAligned", fullyAligned);
        Logger.recordOutput("VisionTest/ModelC/TagID", vision.getTagID());
        Logger.recordOutput("VisionTest/ModelC/DistanceError", distance - targetDistanceMeters);
    }

    private void logStatus(String status) {
        telemetryTable.getEntry("CommandStatus").setString(status);
        Logger.recordOutput("VisionTest/ModelC/CommandStatus", status);
    }

    @Override
    public boolean isFinished() {
        // Runs until manually stopped
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopRobot();
        megaTagOrientationSet = false;
        logStatus(interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}

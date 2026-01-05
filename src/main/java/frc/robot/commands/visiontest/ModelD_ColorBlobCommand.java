package frc.robot.commands.visiontest;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
/*
 * Algae color
 * H: 35 → 65
 * S: 120 → 255
 * V: 80 → 255

 Limelight Setup Tips (this matters more than the numbers)

Turn LEDs ON (force on during tuning)

Tune Hue first, then Saturation, then Value

Enable YUV or HSV view, not RGB

Use Contour filtering:

Min area: ~0.5–1.0%

Aspect ratio: fairly loose (it’s spherical)

Crop the image if algae is always below bumper height
 */
/**
 * ===========================================================================
 * MODEL D: COLOR BLOB HUNT AND SEEK
 * ===========================================================================
 *
 * Adapted from Limelight "Getting in Range" example with color pipeline.
 *
 * PURPOSE:
 * - HUNT MODE: Rotate to find a teal-colored blob when not visible
 * - SEEK MODE: Once visible, rotate to center and drive to target distance
 * - Uses color pipeline (pipeline 1) for blob detection
 *
 * TWO-PHASE OPERATION:
 *
 * PHASE 1 - HUNT:
 * - No target visible
 * - Robot rotates slowly to search for teal blob
 * - Continues until blob enters field of view
 *
 * PHASE 2 - SEEK:
 * - Target visible
 * - Robot rotates to center blob (tx = 0)
 * - Robot drives forward/backward to target distance
 * - Distance estimated from target area (larger area = closer)
 *
 * TEAL COLOR CONFIGURATION:
 * - Configure color pipeline in Limelight web interface (pipeline 1)
 * - Set HSV thresholds for teal: H=85-100, S=100-255, V=100-255 (approximate)
 * - Adjust in Limelight UI for your specific teal color/lighting
 *
 * DISTANCE ESTIMATION:
 * - Uses target area (ta) to estimate distance
 * - Requires calibration: set TARGET_AREA_FOR_DISTANCE at known 1.2m distance
 * - Distance approx = sqrt(knownArea / currentArea) * knownDistance
 *
 * SUCCESS CRITERIA:
 * - TX within tolerance (3.0 degrees)
 * - Target area indicates correct distance
 *
 * TELEMETRY OUTPUT:
 * - VisionTest/ModelD/Mode: HUNT or SEEK
 * - VisionTest/ModelD/TX: Horizontal offset
 * - VisionTest/ModelD/TargetArea: Current blob area
 * - VisionTest/ModelD/EstimatedDistance: Distance estimate from area
 * - VisionTest/ModelD/RotationSpeed, ForwardSpeed: Applied velocities
 *
 * BUTTON BINDING: Shuffleboard "Model D: Color Hunt & Seek" button
 *
 * IMPORTANT: Configure pipeline 1 in Limelight web interface for teal color detection!
 */
public class ModelD_ColorBlobCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID Controllers
    private final PIDController huntRotationPID;    // Hunt mode rotation (slow search)
    private final PIDController seekRotationPID;    // Seek mode rotation (center on target)
    private final PIDController rangePID;           // Distance control

    // Target distance
    private final double targetDistanceMeters;

    // NetworkTables for telemetry
    private final NetworkTable telemetryTable;

    // Hunt mode rotation direction and timing
    private double huntDirection = 1.0;  // 1.0 = clockwise, -1.0 = counter-clockwise
    private final Timer huntTimer = new Timer();
    private static final double HUNT_DIRECTION_CHANGE_SECONDS = 3.0;  // Change direction every 3 seconds

    // Last known target area at calibration distance (tune this value!)
    // Measure target area when blob is at 1.2m distance and update this value
    private static final double CALIBRATION_DISTANCE_METERS = 1.2;

    /**
     * Creates a ColorBlobHuntCommand with default target distance (1.2m).
     */
    public ModelD_ColorBlobCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, VisionConstants.DEFAULT_TARGET_DISTANCE_METERS);
    }

    /**
     * Creates a ColorBlobHuntCommand with custom target distance.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The Limelight vision subsystem
     * @param targetDistanceMeters Target distance from color blob in meters
     */
    public ModelD_ColorBlobCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, double targetDistanceMeters) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;

        // Initialize hunt mode rotation PID (slower, gentler search)
        huntRotationPID = new PIDController(
            VisionConstants.ModelD.HUNT_ROTATION_KP,
            VisionConstants.ModelD.HUNT_ROTATION_KI,
            VisionConstants.ModelD.HUNT_ROTATION_KD
        );

        // Initialize seek mode rotation PID
        seekRotationPID = new PIDController(
            VisionConstants.ModelD.SEEK_ROTATION_KP,
            VisionConstants.ModelD.SEEK_ROTATION_KI,
            VisionConstants.ModelD.SEEK_ROTATION_KD
        );
        seekRotationPID.setTolerance(VisionConstants.ModelD.ROTATION_TOLERANCE_DEGREES);
        seekRotationPID.setSetpoint(VisionConstants.TARGET_TX_CENTERED);
        seekRotationPID.enableContinuousInput(-180, 180);

        // Initialize range PID
        rangePID = new PIDController(
            VisionConstants.ModelD.RANGE_KP,
            VisionConstants.ModelD.RANGE_KI,
            VisionConstants.ModelD.RANGE_KD
        );
        rangePID.setTolerance(VisionConstants.ModelD.DISTANCE_TOLERANCE_METERS);
        rangePID.setSetpoint(targetDistanceMeters);

        // Setup NetworkTables
        telemetryTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.NTKeys.MODEL_D_PREFIX.replace("/", ""));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        huntRotationPID.reset();
        seekRotationPID.reset();
        rangePID.reset();
        // Set vision mode in subsystem
        vision.setVisionMode(VisionSubsystem.VisionMode.COLOR_BLOB_HUNT);
        vision.setAlignmentState(VisionSubsystem.AlignmentState.HUNTING);
        huntDirection = 1.0;
        huntTimer.restart();
        logStatus("STARTED - HUNTING");

        // Switch to color pipeline (pipeline 1)
        // IMPORTANT: Pipeline 1 must be configured in Limelight web interface for teal color detection!
        LimelightHelpers.setPipelineIndex(vision.getName(), VisionConstants.ModelD.COLOR_PIPELINE_INDEX);

        // Log configuration
        telemetryTable.getEntry("TargetDistance").setDouble(targetDistanceMeters);
        telemetryTable.getEntry("ColorPipeline").setDouble(VisionConstants.ModelD.COLOR_PIPELINE_INDEX);
        Logger.recordOutput("VisionTest/ModelD/TargetDistance", targetDistanceMeters);
        Logger.recordOutput("VisionTest/ModelD/ColorPipeline", VisionConstants.ModelD.COLOR_PIPELINE_INDEX);
    }

    @Override
    public void execute() {
        // Check if Limelight sees a target
        boolean hasTarget = LimelightHelpers.getTV(vision.getName());
        double ta = LimelightHelpers.getTA(vision.getName());
        double tx = LimelightHelpers.getTX(vision.getName());

        // Validate target - must have minimum area
        boolean validTarget = hasTarget && ta >= VisionConstants.ModelD.MIN_TARGET_AREA;

        if (!validTarget) {
            // HUNT MODE - No target, rotate to search
            executeHuntMode();
        } else {
            // SEEK MODE - Target found, align and approach
            executeSeekMode(tx, ta);
        }
    }

    /**
     * Hunt mode: Rotate to search for the color blob.
     * Alternates direction periodically to cover full field of view.
     */
    private void executeHuntMode() {
        // Update state machine
        vision.setAlignmentState(VisionSubsystem.AlignmentState.HUNTING);

        // Change hunt direction periodically
        if (huntTimer.hasElapsed(HUNT_DIRECTION_CHANGE_SECONDS)) {
            huntDirection *= -1.0;  // Reverse direction
            huntTimer.restart();
            Logger.recordOutput("VisionTest/ModelD/HuntDirectionChange", huntDirection > 0 ? "CW" : "CCW");
        }

        // Apply constant rotation speed in hunt direction
        double rotationSpeed = VisionConstants.ModelD.HUNT_ROTATION_SPEED_RADPS * huntDirection;

        // Apply to drivetrain - rotation only
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(0.0, 0.0, 0.0, rotationSpeed, 0.0, false, false);
    }

    /**
     * Seek mode: Center on the blob and drive to target distance.
     *
     * @param tx Horizontal offset to target (degrees)
     * @param ta Target area (0-100%)
     */
    private void executeSeekMode(double tx, double ta) {
        // Reset hunt timer when we have a target
        huntTimer.restart();

        // Estimate distance from target area
        // Larger area = closer to target
        // Uses inverse square law approximation: distance = k / sqrt(area)
        double estimatedDistance = estimateDistanceFromArea(ta);

        // Calculate rotation speed to center on target
        double rotationSpeed = -seekRotationPID.calculate(tx);

        // Calculate forward speed to reach target distance
        double forwardSpeed = rangePID.calculate(estimatedDistance);

        // Apply speed limits
        rotationSpeed = MathUtil.clamp(
            rotationSpeed,
            -VisionConstants.ModelD.MAX_ROTATION_SPEED_RADPS,
            VisionConstants.ModelD.MAX_ROTATION_SPEED_RADPS
        );
        forwardSpeed = MathUtil.clamp(
            forwardSpeed,
            -VisionConstants.ModelD.MAX_FORWARD_SPEED_MPS,
            VisionConstants.ModelD.MAX_FORWARD_SPEED_MPS
        );

        // Check alignment
        boolean rotationAligned = seekRotationPID.atSetpoint();
        boolean distanceAligned = rangePID.atSetpoint();

        // Update state machine based on alignment status
        if (rotationAligned && distanceAligned) {
            vision.setAlignmentState(VisionSubsystem.AlignmentState.ALIGNED);
        } else {
            vision.setAlignmentState(VisionSubsystem.AlignmentState.SEEKING);
        }

        // Apply to drivetrain
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(tx, ta, estimatedDistance, rotationSpeed, forwardSpeed, rotationAligned, distanceAligned);
    }

    /**
     * Estimates distance to the color blob based on target area.
     *
     * Uses the principle that target area decreases with the square of distance:
     * area1 / area2 = (distance2 / distance1)^2
     *
     * Solving for distance2:
     * distance2 = distance1 * sqrt(area1 / area2)
     *
     * Where area1 is the calibrated area at the known calibration distance.
     *
     * NOTE: You should calibrate TARGET_AREA_FOR_DISTANCE by measuring the
     * target area when the blob is exactly at 1.2m distance.
     *
     * @param currentArea Current target area from Limelight (0-100%)
     * @return Estimated distance in meters
     */
    private double estimateDistanceFromArea(double currentArea) {
        if (currentArea < 0.01) {
            return Double.MAX_VALUE;  // Avoid division by zero
        }

        double calibrationArea = VisionConstants.ModelD.TARGET_AREA_FOR_DISTANCE;

        // distance = calibrationDistance * sqrt(calibrationArea / currentArea)
        double estimatedDistance = CALIBRATION_DISTANCE_METERS *
            Math.sqrt(calibrationArea / currentArea);

        // Clamp to reasonable range
        return MathUtil.clamp(estimatedDistance, 0.1, 5.0);
    }

    private void stopRobot() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private void logTelemetry(double tx, double ta, double estimatedDistance,
                               double rotationSpeed, double forwardSpeed,
                               boolean rotationAligned, boolean distanceAligned) {
        boolean fullyAligned = rotationAligned && distanceAligned;
        // Get current state from subsystem
        String status = vision.getAlignmentState().name();

        // NetworkTables output
        telemetryTable.getEntry("Mode").setString(status);
        telemetryTable.getEntry("TX").setDouble(tx);
        telemetryTable.getEntry("TargetArea").setDouble(ta);
        telemetryTable.getEntry("EstimatedDistance").setDouble(estimatedDistance);
        telemetryTable.getEntry("RotationSpeed").setDouble(rotationSpeed);
        telemetryTable.getEntry("ForwardSpeed").setDouble(forwardSpeed);
        telemetryTable.getEntry("RotationAligned").setBoolean(rotationAligned);
        telemetryTable.getEntry("DistanceAligned").setBoolean(distanceAligned);
        telemetryTable.getEntry("FullyAligned").setBoolean(fullyAligned);
        telemetryTable.getEntry("HuntDirection").setString(huntDirection > 0 ? "CW" : "CCW");
        telemetryTable.getEntry("DistanceError").setDouble(estimatedDistance - targetDistanceMeters);

        // AdvantageKit logging
        Logger.recordOutput("VisionTest/ModelD/Mode", status);
        Logger.recordOutput("VisionTest/ModelD/TX", tx);
        Logger.recordOutput("VisionTest/ModelD/TargetArea", ta);
        Logger.recordOutput("VisionTest/ModelD/EstimatedDistance", estimatedDistance);
        Logger.recordOutput("VisionTest/ModelD/RotationSpeed", rotationSpeed);
        Logger.recordOutput("VisionTest/ModelD/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("VisionTest/ModelD/RotationAligned", rotationAligned);
        Logger.recordOutput("VisionTest/ModelD/DistanceAligned", distanceAligned);
        Logger.recordOutput("VisionTest/ModelD/FullyAligned", fullyAligned);
        Logger.recordOutput("VisionTest/ModelD/HuntDirection", huntDirection > 0 ? "CW" : "CCW");
        Logger.recordOutput("VisionTest/ModelD/DistanceError", estimatedDistance - targetDistanceMeters);
    }

    private void logStatus(String status) {
        telemetryTable.getEntry("CommandStatus").setString(status);
        Logger.recordOutput("VisionTest/ModelD/CommandStatus", status);
    }

    @Override
    public boolean isFinished() {
        // Runs until manually stopped
        // Or return true when aligned if you want auto-finish:
        // return vision.isAligned();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopRobot();
        huntTimer.stop();

        // Switch back to AprilTag pipeline
        LimelightHelpers.setPipelineIndex(vision.getName(), VisionConstants.ModelD.APRILTAG_PIPELINE_INDEX);

        // Reset state machine when command ends
        vision.resetStateMachine();

        logStatus(interrupted ? "INTERRUPTED" : "COMPLETED");
        Logger.recordOutput("VisionTest/ModelD/PipelineRestored", true);
    }
}

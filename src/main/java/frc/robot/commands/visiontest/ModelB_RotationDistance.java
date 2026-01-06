package frc.robot.commands.visiontest;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.TunableVisionConstants;
import org.littletonrobotics.junction.Logger;

/**
 * ===========================================================================
 * MODEL B: ROTATIONAL ALIGNMENT + X-AXIS DISTANCE
 * ===========================================================================
 *
 * Adapted from Limelight "Aiming and Ranging Simultaneously" example.
 *
 * PURPOSE:
 * - Rotates the robot to center AprilTag on Y-axis (tx = 0)
 * - Simultaneously drives forward/backward to target distance (default: 1.2m)
 * - Robot does NOT strafe (no Y-axis movement)
 *
 * HOW IT WORKS:
 * 1. Reads TX (horizontal offset) and TY (vertical offset) from Limelight
 * 2. Calculates distance using trigonometry: distance = heightDiff / tan(angle)
 * 3. Uses PID controllers for both rotation and range
 * 4. Applies both outputs simultaneously to drivetrain
 *
 * DISTANCE CALCULATION:
 * - Uses camera height, tag height, and TY angle to calculate distance
 * - More accurate when camera is level (CAMERA_ANGLE_DEGREES = 0)
 *
 * SUCCESS CRITERIA:
 * - TX within rotation tolerance (1.5 degrees)
 * - Distance within range tolerance (0.05 meters)
 *
 * TELEMETRY OUTPUT:
 * - VisionTest/ModelB/Status: Current state
 * - VisionTest/ModelB/TX: Horizontal offset
 * - VisionTest/ModelB/Distance: Current distance to tag
 * - VisionTest/ModelB/TargetDistance: Target distance (1.2m)
 * - VisionTest/ModelB/RotationSpeed: Applied rotation rate
 * - VisionTest/ModelB/ForwardSpeed: Applied forward speed
 * - VisionTest/ModelB/RotationAtTarget: Rotation aligned
 * - VisionTest/ModelB/DistanceAtTarget: Distance aligned
 *
 * BUTTON BINDING: Shuffleboard "Model B: Rotation + Range" button
 */
public class ModelB_RotationDistance extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID Controllers
    private final PIDController rotationPID;
    private final PIDController rangePID;

    // Target distance (can be overridden via constructor)
    private final double targetDistanceMeters;

    // NetworkTables for telemetry
    private final NetworkTable telemetryTable;

    /**
     * Creates a RotationalRangeAlignCommand with default target distance (0.75m).
     */
    public ModelB_RotationDistance(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this(drivetrain, vision, VisionConstants.DEFAULT_TARGET_DISTANCE_METERS);
    }

    /**
     * Creates a RotationalRangeAlignCommand with custom target distance.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The Limelight vision subsystem
     * @param targetDistanceMeters Target distance from AprilTag in meters
     */
    public ModelB_RotationDistance(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, double targetDistanceMeters) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;

        // Initialize rotation PID
        rotationPID = new PIDController(
            VisionConstants.ModelB.ROTATION_KP,
            VisionConstants.ModelB.ROTATION_KI,
            VisionConstants.ModelB.ROTATION_KD
        );
        rotationPID.setTolerance(VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES);
        rotationPID.setSetpoint(VisionConstants.TARGET_TX_CENTERED);
        rotationPID.enableContinuousInput(-180, 180);

        // Initialize range/distance PID
        rangePID = new PIDController(
            VisionConstants.ModelB.RANGE_KP,
            VisionConstants.ModelB.RANGE_KI,
            VisionConstants.ModelB.RANGE_KD
        );
        rangePID.setTolerance(VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS);
        rangePID.setSetpoint(targetDistanceMeters);

        // Setup NetworkTables
        telemetryTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.NTKeys.MODEL_B_PREFIX.replace("/", ""));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rangePID.reset();
        // Set vision mode in subsystem
        vision.setVisionMode(VisionSubsystem.VisionMode.ROTATION_RANGE);
        vision.setAlignmentState(VisionSubsystem.AlignmentState.SEARCHING);
        logStatus("STARTED");

        // Log target distance
        telemetryTable.getEntry("TargetDistance").setDouble(targetDistanceMeters);
        Logger.recordOutput("VisionTest/ModelB/TargetDistance", targetDistanceMeters);
    }

    @Override
    public void execute() {
        // Update PID gains from dashboard if changed
        updatePIDGains();

        // Check for valid target
        if (!vision.hasTarget()) {
            // Update state machine
            if (vision.getAlignmentState() == VisionSubsystem.AlignmentState.ALIGNING ||
                vision.getAlignmentState() == VisionSubsystem.AlignmentState.ALIGNED) {
                vision.setAlignmentState(VisionSubsystem.AlignmentState.LOST_TARGET);
            } else {
                vision.setAlignmentState(VisionSubsystem.AlignmentState.SEARCHING);
            }
            stopRobot();
            logTelemetry(0.0, 0.0, 0.0, 0.0, false, false);
            return;
        }

        // Get measurements from Limelight
        double tx = vision.getTX();
        double ty = vision.getTY();

        // Calculate distance to tag using trigonometry
        // distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
        double heightDiff = VisionConstants.TAG_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionConstants.CAMERA_ANGLE_DEGREES + ty;

        // Prevent division by zero or near-zero
        double currentDistance;
        if (Math.abs(angleToTag) < 0.5) {
            currentDistance = targetDistanceMeters; // Fallback
        } else {
            currentDistance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));
        }

        // Calculate rotation speed
        double rotationSpeed = VisionConstants.ROTATION_DIRECTION_MULTIPLIER * rotationPID.calculate(tx);

        // Calculate forward speed (positive = move forward toward tag)
        // Note: If current distance > target, we need to move forward (positive)
        //       If current distance < target, we need to move backward (negative)
        double forwardSpeed = rangePID.calculate(currentDistance);

        // Apply speed limits
        double maxRotation = TunableVisionConstants.ModelB.getMaxRotationSpeed();
        double maxForward = TunableVisionConstants.ModelB.getMaxForwardSpeed();
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotation, maxRotation);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -maxForward, maxForward);

        // Check alignment status
        boolean rotationAligned = rotationPID.atSetpoint();
        boolean distanceAligned = rangePID.atSetpoint();

        // Update state machine based on alignment status
        vision.setAlignmentState((rotationAligned && distanceAligned) ?
            VisionSubsystem.AlignmentState.ALIGNED :
            VisionSubsystem.AlignmentState.ALIGNING);

        // Apply to drivetrain - rotation + forward, no lateral movement
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(tx, currentDistance, rotationSpeed, forwardSpeed, rotationAligned, distanceAligned);
    }

    /**
     * Updates PID gains from tunable constants if they've changed.
     * Called every execute() cycle to allow live tuning.
     */
    private void updatePIDGains() {
        // Update rotation PID
        if (TunableVisionConstants.ModelB.rotationPIDHasChanged()) {
            rotationPID.setPID(
                TunableVisionConstants.ModelB.getRotationKp(),
                TunableVisionConstants.ModelB.getRotationKi(),
                TunableVisionConstants.ModelB.getRotationKd()
            );
        }

        if (TunableVisionConstants.ModelB.rotationToleranceHasChanged()) {
            rotationPID.setTolerance(TunableVisionConstants.ModelB.getRotationTolerance());
        }

        // Update range PID
        if (TunableVisionConstants.ModelB.rangePIDHasChanged()) {
            rangePID.setPID(
                TunableVisionConstants.ModelB.getRangeKp(),
                TunableVisionConstants.ModelB.getRangeKi(),
                TunableVisionConstants.ModelB.getRangeKd()
            );
        }

        if (TunableVisionConstants.ModelB.distanceToleranceHasChanged()) {
            rangePID.setTolerance(TunableVisionConstants.ModelB.getDistanceTolerance());
        }

        if (TunableVisionConstants.ModelB.targetDistanceHasChanged()) {
            rangePID.setSetpoint(TunableVisionConstants.ModelB.getTargetDistance());
        }
    }

    private void stopRobot() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private void logTelemetry(double tx, double distance, double rotationSpeed,
                               double forwardSpeed, boolean rotationAligned, boolean distanceAligned) {
        // Get current state from subsystem
        String status = vision.getAlignmentState().name();

        // NetworkTables output
        telemetryTable.getEntry("Status").setString(status);
        telemetryTable.getEntry("TX").setDouble(tx);
        telemetryTable.getEntry("Distance").setDouble(distance);
        telemetryTable.getEntry("RotationSpeed").setDouble(rotationSpeed);
        telemetryTable.getEntry("ForwardSpeed").setDouble(forwardSpeed);
        telemetryTable.getEntry("RotationAtTarget").setBoolean(rotationAligned);
        telemetryTable.getEntry("DistanceAtTarget").setBoolean(distanceAligned);
        telemetryTable.getEntry("FullyAligned").setBoolean(rotationAligned && distanceAligned);
        telemetryTable.getEntry("TagID").setDouble(vision.getTagID());
        telemetryTable.getEntry("DistanceError").setDouble(distance - targetDistanceMeters);

        // AdvantageKit logging
        Logger.recordOutput("VisionTest/ModelB/Status", status);
        Logger.recordOutput("VisionTest/ModelB/TX", tx);
        Logger.recordOutput("VisionTest/ModelB/Distance", distance);
        Logger.recordOutput("VisionTest/ModelB/RotationSpeed", rotationSpeed);
        Logger.recordOutput("VisionTest/ModelB/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("VisionTest/ModelB/RotationAtTarget", rotationAligned);
        Logger.recordOutput("VisionTest/ModelB/DistanceAtTarget", distanceAligned);
        Logger.recordOutput("VisionTest/ModelB/FullyAligned", rotationAligned && distanceAligned);
        Logger.recordOutput("VisionTest/ModelB/TagID", vision.getTagID());
        Logger.recordOutput("VisionTest/ModelB/DistanceError", distance - targetDistanceMeters);
    }

    private void logStatus(String status) {
        telemetryTable.getEntry("CommandStatus").setString(status);
        Logger.recordOutput("VisionTest/ModelB/CommandStatus", status);
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
        // Reset state machine when command ends
        vision.resetStateMachine();
        logStatus(interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}

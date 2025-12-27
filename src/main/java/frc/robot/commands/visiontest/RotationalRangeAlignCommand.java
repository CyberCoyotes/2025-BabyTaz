package frc.robot.commands.visiontest;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightVision;
import frc.robot.subsystems.vision.VisionTestConstants;
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
public class RotationalRangeAlignCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightVision vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID Controllers
    private final PIDController rotationPID;
    private final PIDController rangePID;

    // Target distance (can be overridden via constructor)
    private final double targetDistanceMeters;

    // NetworkTables for telemetry
    private final NetworkTable telemetryTable;

    // Status tracking
    private enum AlignmentStatus {
        SEARCHING,      // No target visible
        ALIGNING,       // Target visible, aligning
        ALIGNED         // Both rotation and distance within tolerance
    }
    private AlignmentStatus currentStatus = AlignmentStatus.SEARCHING;

    /**
     * Creates a RotationalRangeAlignCommand with default target distance (1.2m).
     */
    public RotationalRangeAlignCommand(CommandSwerveDrivetrain drivetrain, LimelightVision vision) {
        this(drivetrain, vision, VisionTestConstants.DEFAULT_TARGET_DISTANCE_METERS);
    }

    /**
     * Creates a RotationalRangeAlignCommand with custom target distance.
     *
     * @param drivetrain The swerve drivetrain
     * @param vision The Limelight vision subsystem
     * @param targetDistanceMeters Target distance from AprilTag in meters
     */
    public RotationalRangeAlignCommand(CommandSwerveDrivetrain drivetrain, LimelightVision vision, double targetDistanceMeters) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;

        // Initialize rotation PID
        rotationPID = new PIDController(
            VisionTestConstants.ModelB.ROTATION_KP,
            VisionTestConstants.ModelB.ROTATION_KI,
            VisionTestConstants.ModelB.ROTATION_KD
        );
        rotationPID.setTolerance(VisionTestConstants.ModelB.ROTATION_TOLERANCE_DEGREES);
        rotationPID.setSetpoint(VisionTestConstants.TARGET_TX_CENTERED);
        rotationPID.enableContinuousInput(-180, 180);

        // Initialize range/distance PID
        rangePID = new PIDController(
            VisionTestConstants.ModelB.RANGE_KP,
            VisionTestConstants.ModelB.RANGE_KI,
            VisionTestConstants.ModelB.RANGE_KD
        );
        rangePID.setTolerance(VisionTestConstants.ModelB.DISTANCE_TOLERANCE_METERS);
        rangePID.setSetpoint(targetDistanceMeters);

        // Setup NetworkTables
        telemetryTable = NetworkTableInstance.getDefault()
            .getTable(VisionTestConstants.NTKeys.MODEL_B_PREFIX.replace("/", ""));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        rangePID.reset();
        currentStatus = AlignmentStatus.SEARCHING;
        logStatus("STARTED");

        // Log target distance
        telemetryTable.getEntry("TargetDistance").setDouble(targetDistanceMeters);
        Logger.recordOutput("VisionTest/ModelB/TargetDistance", targetDistanceMeters);
    }

    @Override
    public void execute() {
        // Check for valid target
        if (!vision.hasTarget()) {
            currentStatus = AlignmentStatus.SEARCHING;
            stopRobot();
            logTelemetry(0.0, 0.0, 0.0, 0.0, false, false);
            return;
        }

        // Get measurements from Limelight
        double tx = vision.getTX();
        double ty = vision.getTY();

        // Calculate distance to tag using trigonometry
        // distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
        double heightDiff = VisionTestConstants.TAG_HEIGHT_METERS - VisionTestConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionTestConstants.CAMERA_ANGLE_DEGREES + ty;

        // Prevent division by zero or near-zero
        double currentDistance;
        if (Math.abs(angleToTag) < 0.5) {
            currentDistance = targetDistanceMeters; // Fallback
        } else {
            currentDistance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));
        }

        // Calculate rotation speed
        double rotationSpeed = rotationPID.calculate(tx);

        // Calculate forward speed (positive = move forward toward tag)
        // Note: If current distance > target, we need to move forward (positive)
        //       If current distance < target, we need to move backward (negative)
        double forwardSpeed = rangePID.calculate(currentDistance);

        // Apply speed limits
        rotationSpeed = MathUtil.clamp(
            rotationSpeed,
            -VisionTestConstants.ModelB.MAX_ROTATION_SPEED_RADPS,
            VisionTestConstants.ModelB.MAX_ROTATION_SPEED_RADPS
        );
        forwardSpeed = MathUtil.clamp(
            forwardSpeed,
            -VisionTestConstants.ModelB.MAX_FORWARD_SPEED_MPS,
            VisionTestConstants.ModelB.MAX_FORWARD_SPEED_MPS
        );

        // Check alignment status
        boolean rotationAligned = rotationPID.atSetpoint();
        boolean distanceAligned = rangePID.atSetpoint();
        currentStatus = (rotationAligned && distanceAligned) ? AlignmentStatus.ALIGNED : AlignmentStatus.ALIGNING;

        // Apply to drivetrain - rotation + forward, no lateral movement
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(tx, currentDistance, rotationSpeed, forwardSpeed, rotationAligned, distanceAligned);
    }

    private void stopRobot() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private void logTelemetry(double tx, double distance, double rotationSpeed,
                               double forwardSpeed, boolean rotationAligned, boolean distanceAligned) {
        // NetworkTables output
        telemetryTable.getEntry("Status").setString(currentStatus.name());
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
        Logger.recordOutput("VisionTest/ModelB/Status", currentStatus.name());
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
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopRobot();
        logStatus(interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}

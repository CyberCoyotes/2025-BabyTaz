package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision18.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to an AprilTag (IDs 1-36).
 *
 * This command:
 * - Aligns the robot forward/backward (X-axis) to maintain target distance
 * - Aligns the robot left/right (Y-axis) to center on the tag
 * - Rotates the robot to be perpendicular to the AprilTag
 * - Uses modest speeds for safety during initial testing
 * - Can be interrupted by button press
 *
 * Alignment Target Values:
 * - targetDistance: Desired distance from tag (meters)
 * - targetLateralOffset: Desired left/right offset (0 = centered)
 * - targetRotation: Desired rotation relative to tag (0 = perpendicular)
 *
 * Success Thresholds:
 * - Forward tolerance: Within 0.10 meters (10 cm) of target distance
 * - Lateral tolerance: Within 0.05 meters (5 cm) of center
 * - Rotation tolerance: Within 2.0 degrees of perpendicular
 */
public class AlignToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // PID Controllers for three-axis alignment
    private final PIDController forwardController;   // X-axis: Forward/Backward
    private final PIDController lateralController;   // Y-axis: Left/Right
    private final PIDController rotationController;  // Rotation: Turn to face tag

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    // ====== ALIGNMENT TARGET VALUES ======
    // These define where the robot should position itself
    private static final double TARGET_DISTANCE_METERS = 1.0;      // Distance from AprilTag (forward/back)
    private static final double TARGET_LATERAL_OFFSET_METERS = 0.0; // Left/right offset (0 = centered)
    private static final double TARGET_ROTATION_DEGREES = 0.0;      // Rotation relative to tag (0 = perpendicular)

    // ====== ALIGNMENT SUCCESS THRESHOLDS ======
    // Robot is considered "aligned" when within these tolerances
    private static final double FORWARD_TOLERANCE_METERS = 0.10;    // Forward/back tolerance (10 cm)
    private static final double LATERAL_TOLERANCE_METERS = 0.05;    // Left/right tolerance (5 cm)
    private static final double ROTATION_TOLERANCE_DEGREES = 2.0;   // Rotation tolerance (2 degrees)

    // ====== SPEED LIMITS ======
    // Increased speeds for improved alignment performance (25% faster)
    private static final double MAX_FORWARD_SPEED_MPS = 0.625;      // Max forward speed (meters/second)
    private static final double MAX_LATERAL_SPEED_MPS = 0.625;      // Max lateral speed (meters/second)
    private static final double MAX_ROTATION_SPEED_RADPS = 0.9375;  // Max rotation speed (radians/second)

    // ====== PID TUNING VALUES ======
    // Forward/Backward Control (X-axis)
    private static final double FORWARD_KP = 1.0;
    private static final double FORWARD_KI = 0.0;
    private static final double FORWARD_KD = 0.0;

    // Left/Right Control (Y-axis)
    private static final double LATERAL_KP = 0.3;
    private static final double LATERAL_KI = 0.0;
    private static final double LATERAL_KD = 0.0;

    // Rotation Control (perpendicular to tag)
    private static final double ROTATION_KP = 0.08;
    private static final double ROTATION_KI = 0.0;
    private static final double ROTATION_KD = 0.0;

    // ====== CAMERA CONFIGURATION ======
    // Physical camera mounting position and angle
    private static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(12.0);  // Height of camera
    private static final double CAMERA_ANGLE_DEGREES = 0.0;                          // Camera tilt angle

    // AprilTag physical dimensions
    private static final double APRILTAG_HEIGHT_METERS = Units.inchesToMeters(25.5); // Height of AprilTag center

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param drivetrain The swerve drivetrain subsystem
     * @param vision The vision subsystem with Limelight
     */
    public AlignToAprilTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Initialize PID controllers with tuning values
        forwardController = new PIDController(FORWARD_KP, FORWARD_KI, FORWARD_KD);
        forwardController.setTolerance(FORWARD_TOLERANCE_METERS);

        lateralController = new PIDController(LATERAL_KP, LATERAL_KI, LATERAL_KD);
        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);

        rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEGREES);
        rotationController.enableContinuousInput(-180, 180); // Handle angle wrapping

        addRequirements(drivetrain);
        setupLogging();
    }

    @Override
    public void initialize() {
        // Reset PID controllers when command starts
        forwardController.reset();
        lateralController.reset();
        rotationController.reset();

        Logger.recordOutput("AprilTagAlign/Status", "STARTED");
        SmartDashboard.putString("AprilTagAlign/Status", "STARTED");
    }

    @Override
    public void execute() {
        // Check if Limelight sees a valid AprilTag
        if (!LimelightHelpers.getTV(vision.getName())) {
            handleNoTarget();
            return;
        }

        // Get current AprilTag ID
        int tagId = (int) LimelightHelpers.getFiducialID(vision.getName());

        // Only process valid AprilTag IDs (1-36 for 36h11 family)
        if (tagId < 1 || tagId > 36) {
            handleNoTarget();
            return;
        }

        // Get Limelight measurements
        double tx = LimelightHelpers.getTX(vision.getName());  // Horizontal offset (degrees)
        double ty = LimelightHelpers.getTY(vision.getName());  // Vertical offset (degrees)

        // Calculate current distance from target using trigonometry
        double currentDistance = calculateDistanceToTag(ty);

        // Calculate control outputs using PID controllers
        // Forward/Backward: Move to target distance
        double forwardSpeed = forwardController.calculate(currentDistance, TARGET_DISTANCE_METERS);

        // Left/Right: Center on tag (tx = 0 means centered)
        double lateralSpeed = lateralController.calculate(tx, TARGET_LATERAL_OFFSET_METERS);

        // Rotation: Make robot perpendicular to tag (tx = 0 means perpendicular)
        // Note: For perpendicular alignment, we use tx (horizontal offset) to rotate
        double rotationSpeed = rotationController.calculate(tx, TARGET_ROTATION_DEGREES);

        // Apply speed limits for safety
        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_FORWARD_SPEED_MPS, MAX_FORWARD_SPEED_MPS);
        lateralSpeed = MathUtil.clamp(lateralSpeed, -MAX_LATERAL_SPEED_MPS, MAX_LATERAL_SPEED_MPS);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED_RADPS, MAX_ROTATION_SPEED_RADPS);

        // Log telemetry data
        logAlignmentData(tagId, tx, ty, currentDistance, forwardSpeed, lateralSpeed, rotationSpeed);

        // Apply control to drivetrain (robot-centric)
        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)       // Forward/Backward movement
            .withVelocityY(lateralSpeed)       // Left/Right strafe
            .withRotationalRate(rotationSpeed)); // Rotation
    }

    /**
     * Handles the case when no valid target is detected.
     * Stops the robot and updates status.
     */
    private void handleNoTarget() {
        drivetrain.stopDrive();
        Logger.recordOutput("AprilTagAlign/Status", "NO TARGET");
        SmartDashboard.putString("AprilTagAlign/Status", "NO TARGET");
    }

    /**
     * Calculates distance to AprilTag using camera angle and vertical offset.
     *
     * Uses trigonometry: distance = (heightDifference) / tan(angle)
     *
     * @param ty Vertical angle offset from Limelight (degrees)
     * @return Distance to tag in meters
     */
    private double calculateDistanceToTag(double ty) {
        double heightDifference = APRILTAG_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
        double angleToTarget = CAMERA_ANGLE_DEGREES + ty;

        // Prevent division by zero
        if (Math.abs(angleToTarget) < 0.1) {
            return TARGET_DISTANCE_METERS; // Return target distance as fallback
        }

        return Math.abs(heightDifference / Math.tan(Math.toRadians(angleToTarget)));
    }

    /**
     * Logs alignment data to SmartDashboard and AdvantageKit.
     */
    private void logAlignmentData(int tagId, double tx, double ty, double distance,
                                   double forwardSpeed, double lateralSpeed, double rotationSpeed) {
        // SmartDashboard logging for driver feedback
        SmartDashboard.putNumber("AprilTagAlign/TagID", tagId);
        SmartDashboard.putNumber("AprilTagAlign/TX", tx);
        SmartDashboard.putNumber("AprilTagAlign/TY", ty);
        SmartDashboard.putNumber("AprilTagAlign/Distance", distance);
        SmartDashboard.putNumber("AprilTagAlign/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("AprilTagAlign/LateralSpeed", lateralSpeed);
        SmartDashboard.putNumber("AprilTagAlign/RotationSpeed", rotationSpeed);

        // Show which axes are aligned
        SmartDashboard.putBoolean("AprilTagAlign/ForwardAligned", forwardController.atSetpoint());
        SmartDashboard.putBoolean("AprilTagAlign/LateralAligned", lateralController.atSetpoint());
        SmartDashboard.putBoolean("AprilTagAlign/RotationAligned", rotationController.atSetpoint());
        SmartDashboard.putBoolean("AprilTagAlign/FullyAligned", isAligned());

        // AdvantageKit logging for detailed analysis
        Logger.recordOutput("AprilTagAlign/TagID", tagId);
        Logger.recordOutput("AprilTagAlign/TX", tx);
        Logger.recordOutput("AprilTagAlign/TY", ty);
        Logger.recordOutput("AprilTagAlign/Distance", distance);
        Logger.recordOutput("AprilTagAlign/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("AprilTagAlign/LateralSpeed", lateralSpeed);
        Logger.recordOutput("AprilTagAlign/RotationSpeed", rotationSpeed);
        Logger.recordOutput("AprilTagAlign/FullyAligned", isAligned());

        // Log error values
        Logger.recordOutput("AprilTagAlign/ForwardError", distance - TARGET_DISTANCE_METERS);
        Logger.recordOutput("AprilTagAlign/LateralError", tx - TARGET_LATERAL_OFFSET_METERS);
        Logger.recordOutput("AprilTagAlign/RotationError", tx - TARGET_ROTATION_DEGREES);
    }

    /**
     * Sets up metadata logging for AdvantageKit.
     */
    private void setupLogging() {
        Logger.recordMetadata("AprilTagAlign/Description",
            "Three-axis AprilTag alignment (X, Y, Rotation)");
        Logger.recordMetadata("AprilTagAlign/TargetDistance",
            String.format("%.2f meters", TARGET_DISTANCE_METERS));
        Logger.recordMetadata("AprilTagAlign/ForwardPID",
            String.format("kP: %.3f, kI: %.3f, kD: %.3f", FORWARD_KP, FORWARD_KI, FORWARD_KD));
        Logger.recordMetadata("AprilTagAlign/LateralPID",
            String.format("kP: %.3f, kI: %.3f, kD: %.3f", LATERAL_KP, LATERAL_KI, LATERAL_KD));
        Logger.recordMetadata("AprilTagAlign/RotationPID",
            String.format("kP: %.3f, kI: %.3f, kD: %.3f", ROTATION_KP, ROTATION_KI, ROTATION_KD));
    }

    /**
     * Checks if robot is fully aligned on all three axes.
     *
     * @return true if forward, lateral, and rotation are all within tolerance
     */
    private boolean isAligned() {
        return forwardController.atSetpoint()
            && lateralController.atSetpoint()
            && rotationController.atSetpoint();
    }

    /**
     * Command finishes when robot is fully aligned.
     * Note: This can be interrupted by button press override.
     */
    @Override
    public boolean isFinished() {
        return isAligned();
    }

    /**
     * Cleanup when command ends (either completed or interrupted).
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();

        if (interrupted) {
            Logger.recordOutput("AprilTagAlign/Status", "INTERRUPTED");
            SmartDashboard.putString("AprilTagAlign/Status", "INTERRUPTED");
        } else {
            Logger.recordOutput("AprilTagAlign/Status", "COMPLETED");
            SmartDashboard.putString("AprilTagAlign/Status", "COMPLETED");
        }
    }
}

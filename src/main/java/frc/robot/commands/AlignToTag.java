package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightVision;
import org.littletonrobotics.junction.Logger;

/**
 * Aligns robot to AprilTag on 3 axes:
 * - Forward/Backward (X): Moves to target distance from tag
 * - Left/Right (Y): Centers horizontally on tag
 * - Rotation: Turns to face tag perpendicular
 *
 * Uses simple PID control with Limelight tx/ty values.
 */
public class AlignToTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightVision vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // Target values
    private static final double TARGET_DISTANCE_METERS = 1.0;  // How far from tag to stop
    private static final double TARGET_TX = 0.0;  // Centered horizontally

    // Camera configuration for distance calculation
    private static final double CAMERA_HEIGHT_METERS = 0.50;  // 50 cm from floor to camera lens
    private static final double CAMERA_OFF_CENTER_METERS = 0.00;  // Camera centered on robot
    private static final double CAMERA_ANGLE_DEGREES = 0.0; // Mounted flat (degrees from horizontal, positive = tilted up)
    private static final double TAG_HEIGHT_METERS = 0.40;  // 40 cm - current test tag height from floor


    // PID Controllers
    private final PIDController forwardPID;    // Controls forward/back movement
    private final PIDController lateralPID;    // Controls left/right strafe
    private final PIDController rotationPID;   // Controls rotation

    // PID Tuning - Forward/Backward
    private static final double FORWARD_KP = 1.2;
    private static final double FORWARD_KI = 0.0;
    private static final double FORWARD_KD = 0.1;
    private static final double FORWARD_TOLERANCE = 0.1;  // 10cm

    // PID Tuning - Left/Right
    private static final double LATERAL_KP = 0.04;  // Start gentle
    private static final double LATERAL_KI = 0.0;
    private static final double LATERAL_KD = 0.0;
    private static final double LATERAL_TOLERANCE = 2.0;  // 2 degrees

    // PID Tuning - Rotation
    private static final double ROTATION_KP = 0.05;  // Start gentle
    private static final double ROTATION_KI = 0.0;
    private static final double ROTATION_KD = 0.0;
    private static final double ROTATION_TOLERANCE = 2.0;  // 2 degrees

    // Speed limits (m/s and rad/s)
    private static final double MAX_FORWARD_SPEED = 0.8;
    private static final double MAX_LATERAL_SPEED = 0.5;
    private static final double MAX_ROTATION_SPEED = 1.0;

    public AlignToTag(CommandSwerveDrivetrain drivetrain, LimelightVision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Initialize PID controllers
        forwardPID = new PIDController(FORWARD_KP, FORWARD_KI, FORWARD_KD);
        forwardPID.setTolerance(FORWARD_TOLERANCE);

        lateralPID = new PIDController(LATERAL_KP, LATERAL_KI, LATERAL_KD);
        lateralPID.setTolerance(LATERAL_TOLERANCE);

        rotationPID = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        rotationPID.setTolerance(ROTATION_TOLERANCE);
        rotationPID.enableContinuousInput(-180, 180);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        forwardPID.reset();
        lateralPID.reset();
        rotationPID.reset();

        Logger.recordOutput("AlignToTag/Active", true);
        Logger.recordOutput("AlignToTag/Status", "STARTED");
    }

    @Override
    public void execute() {
        // Check if we can see a target
        if (!vision.hasTarget()) {
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
            Logger.recordOutput("AlignToTag/Status", "NO TARGET");
            return;
        }

        // Get measurements from Limelight
        double tx = vision.getTX();  // Horizontal offset (degrees)
        double ty = vision.getTY();  // Vertical offset (degrees)
        int tagID = vision.getTagID();

        // Calculate distance to tag using trigonometry
        double heightDiff = TAG_HEIGHT_METERS - CAMERA_HEIGHT_METERS;
        double angleToTag = CAMERA_ANGLE_DEGREES + ty;
        double currentDistance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));

        // Calculate control outputs
        // Forward: Use distance to maintain target distance
        double forwardSpeed = -forwardPID.calculate(currentDistance, TARGET_DISTANCE_METERS);

        // Lateral: Use tx to center on tag (strafe left/right)
        double lateralSpeed = -lateralPID.calculate(tx, TARGET_TX);

        // Rotation: Use tx to rotate and face tag
        double rotationSpeed = -rotationPID.calculate(tx, TARGET_TX);

        // Apply speed limits
        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED);
        lateralSpeed = MathUtil.clamp(lateralSpeed, -MAX_LATERAL_SPEED, MAX_LATERAL_SPEED);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        // Send control to drivetrain (robot-centric)
        drivetrain.setControl(driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(lateralSpeed)
            .withRotationalRate(rotationSpeed));

        // Logging
        Logger.recordOutput("AlignToTag/Status", "ALIGNING");
        Logger.recordOutput("AlignToTag/TagID", tagID);
        Logger.recordOutput("AlignToTag/TX", tx);
        Logger.recordOutput("AlignToTag/TY", ty);
        Logger.recordOutput("AlignToTag/Distance", currentDistance);
        Logger.recordOutput("AlignToTag/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("AlignToTag/LateralSpeed", lateralSpeed);
        Logger.recordOutput("AlignToTag/RotationSpeed", rotationSpeed);
        Logger.recordOutput("AlignToTag/ForwardAtTarget", forwardPID.atSetpoint());
        Logger.recordOutput("AlignToTag/LateralAtTarget", lateralPID.atSetpoint());
        Logger.recordOutput("AlignToTag/RotationAtTarget", rotationPID.atSetpoint());
    }

    @Override
    public boolean isFinished() {
        // Finish when all three axes are aligned
        return forwardPID.atSetpoint()
            && lateralPID.atSetpoint()
            && rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

        Logger.recordOutput("AlignToTag/Active", false);
        Logger.recordOutput("AlignToTag/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}

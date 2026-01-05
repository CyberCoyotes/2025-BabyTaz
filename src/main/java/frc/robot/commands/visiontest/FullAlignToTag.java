package frc.robot.commands.visiontest;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.TunableVisionConstants;

import org.littletonrobotics.junction.Logger;

/**
 * Aligns robot to AprilTag on 3 axes:
 * - Forward/Backward (X): Moves to target distance from tag
 * - Left/Right (Y): Centers horizontally on tag
 * - Rotation: Turns to face tag perpendicular
 *
 * Uses simple PID control with Limelight tx/ty values.
 */
public class FullAlignToTag extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // Target values
    @SuppressWarnings("unused")
    private static final double TARGET_DISTANCE_METERS = VisionConstants.DEFAULT_TARGET_DISTANCE_METERS;  // How far from tag to stop
    private static final double TARGET_TX = VisionConstants.TARGET_TX_CENTERED;


    // PID Controllers
    private final PIDController forwardPID;    // Controls forward/back movement
    private final PIDController lateralPID;    // Controls left/right strafe
    private final PIDController rotationPID;   // Controls rotation

    public FullAlignToTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Initialize PID controllers with tunable constants
        forwardPID = new PIDController(
            TunableVisionConstants.Main.FORWARD_KP.get(),
            TunableVisionConstants.Main.FORWARD_KI.get(),
            TunableVisionConstants.Main.FORWARD_KD.get()
        );
        forwardPID.setTolerance(TunableVisionConstants.Main.FORWARD_TOLERANCE.get());

        lateralPID = new PIDController(
            TunableVisionConstants.Main.LATERAL_KP.get(),
            TunableVisionConstants.Main.LATERAL_KI.get(),
            TunableVisionConstants.Main.LATERAL_KD.get()
        );
        lateralPID.setTolerance(TunableVisionConstants.Main.LATERAL_TOLERANCE.get());

        rotationPID = new PIDController(
            TunableVisionConstants.Main.ROTATION_KP.get(),
            TunableVisionConstants.Main.ROTATION_KI.get(),
            TunableVisionConstants.Main.ROTATION_KD.get()
        );
        rotationPID.setTolerance(TunableVisionConstants.Main.ROTATION_TOLERANCE.get());
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
        // Update PID gains from dashboard if changed
        updatePIDGains();

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
        double heightDiff = VisionConstants.TAG_HEIGHT_METERS - VisionConstants.CAMERA_HEIGHT_METERS;
        double angleToTag = VisionConstants.CAMERA_ANGLE_DEGREES + ty;
        double currentDistance = Math.abs(heightDiff / Math.tan(Math.toRadians(angleToTag)));

        // Calculate control outputs
        // Forward: Use distance to maintain target distance (positive = move forward)
        double targetDistance = TunableVisionConstants.Main.TARGET_DISTANCE.get();
        double forwardSpeed = forwardPID.calculate(currentDistance, targetDistance);

        // Lateral: Use tx to center on tag (strafe left/right)
        // Apply deadband to prevent small corrections from causing arc
        double lateralSpeed = 0.0;
        double lateralTolerance = TunableVisionConstants.Main.LATERAL_TOLERANCE.get();
        if (Math.abs(tx) > lateralTolerance) {
            lateralSpeed = lateralPID.calculate(tx, TARGET_TX);
        }

        // Rotation: Use tx to rotate and face tag
        double rotationSpeed = VisionConstants.ROTATION_DIRECTION_MULTIPLIER * rotationPID.calculate(tx, TARGET_TX);

        // Apply speed limits
        double maxForward = TunableVisionConstants.Main.MAX_FORWARD_SPEED.get();
        double maxLateral = TunableVisionConstants.Main.MAX_LATERAL_SPEED.get();
        double maxRotation = TunableVisionConstants.Main.MAX_ROTATION_SPEED.get();
        forwardSpeed = MathUtil.clamp(forwardSpeed, -maxForward, maxForward);
        lateralSpeed = MathUtil.clamp(lateralSpeed, -maxLateral, maxLateral);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotation, maxRotation);

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

    /**
     * Updates PID gains from tunable constants if they've changed.
     * Called every execute() cycle to allow live tuning.
     */
    private void updatePIDGains() {
        // Update forward PID
        if (TunableVisionConstants.Main.FORWARD_KP.hasChanged() ||
            TunableVisionConstants.Main.FORWARD_KI.hasChanged() ||
            TunableVisionConstants.Main.FORWARD_KD.hasChanged()) {

            forwardPID.setPID(
                TunableVisionConstants.Main.FORWARD_KP.get(),
                TunableVisionConstants.Main.FORWARD_KI.get(),
                TunableVisionConstants.Main.FORWARD_KD.get()
            );
        }

        if (TunableVisionConstants.Main.FORWARD_TOLERANCE.hasChanged()) {
            forwardPID.setTolerance(TunableVisionConstants.Main.FORWARD_TOLERANCE.get());
        }

        // Update lateral PID
        if (TunableVisionConstants.Main.LATERAL_KP.hasChanged() ||
            TunableVisionConstants.Main.LATERAL_KI.hasChanged() ||
            TunableVisionConstants.Main.LATERAL_KD.hasChanged()) {

            lateralPID.setPID(
                TunableVisionConstants.Main.LATERAL_KP.get(),
                TunableVisionConstants.Main.LATERAL_KI.get(),
                TunableVisionConstants.Main.LATERAL_KD.get()
            );
        }

        if (TunableVisionConstants.Main.LATERAL_TOLERANCE.hasChanged()) {
            lateralPID.setTolerance(TunableVisionConstants.Main.LATERAL_TOLERANCE.get());
        }

        // Update rotation PID
        if (TunableVisionConstants.Main.ROTATION_KP.hasChanged() ||
            TunableVisionConstants.Main.ROTATION_KI.hasChanged() ||
            TunableVisionConstants.Main.ROTATION_KD.hasChanged()) {

            rotationPID.setPID(
                TunableVisionConstants.Main.ROTATION_KP.get(),
                TunableVisionConstants.Main.ROTATION_KI.get(),
                TunableVisionConstants.Main.ROTATION_KD.get()
            );
        }

        if (TunableVisionConstants.Main.ROTATION_TOLERANCE.hasChanged()) {
            rotationPID.setTolerance(TunableVisionConstants.Main.ROTATION_TOLERANCE.get());
        }
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

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

// https://claude.ai/chat/768557cd-c3ee-4420-a115-f7c3dc7bb6c8

// This command will:
// Use ProfiledPID control for both rotation and strafing
// Validate AprilTag IDs (1-22)
// Allow live PID tuning via SmartDashboard
// Provide telemetry for debugging
// Use combined rotation and strafe movements to center on the target
// Stop when either no target is visible or the robot is centered and squared up
// You can tune the PID values and speed limits in the Constraints class to match your robot's specific characteristics and requirements.

public class VisionImprovedCenterCommand_v3 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;

    // Add to class fields
    private final SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(1.0); // More gentle ramping
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(1.0);

    // Motion constraints
    private static final class Constraints {
        static final double MAX_VELOCITY = 0.15; // m/s
        static final double MAX_ACCELERATION = 0.5; // m/s²
        static final double MAX_ROTATION_VELOCITY = 1.0; // rad/s
        /* */

        static final double MAX_ROTATION_ACCELERATION = 3.0; // rad/s²

        // Tolerances
        static final double POSITION_TOLERANCE = 0.15; // meters
        static final double ROTATION_TOLERANCE = 0.04; // radians

        // PID Gains
        static final double TRANSLATION_P = 0.08;
        static final double TRANSLATION_I = 0.00;
        static final double TRANSLATION_D = 0.01;
        static final double ROTATION_P = 1.0;
        static final double ROTATION_I = 0.0;
        static final double ROTATION_D = 0.0;
    }

    // Profiled PID Controllers
    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController rotationController;

    public VisionImprovedCenterCommand_v3(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.RobotCentric();

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(Constraints.MAX_VELOCITY,
                Constraints.MAX_ACCELERATION);

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                Constraints.MAX_ROTATION_VELOCITY, Constraints.MAX_ROTATION_ACCELERATION);

        strafeController = new ProfiledPIDController(
                Constraints.TRANSLATION_P,
                Constraints.TRANSLATION_I,
                Constraints.TRANSLATION_D,
                translationConstraints,
                0.02 // 50Hz update rate
        );

        rotationController = new ProfiledPIDController(
                Constraints.ROTATION_P,
                Constraints.ROTATION_I,
                Constraints.ROTATION_D,
                rotationConstraints,
                0.02);

        configurePIDControllers();
        addRequirements(vision, drivetrain);
    }

    private void configurePIDControllers() {
        strafeController.setTolerance(Constraints.POSITION_TOLERANCE);
        rotationController.setTolerance(Constraints.ROTATION_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        strafeController.reset(0, 0);
        rotationController.reset(0, 0);
    }

    @Override
    public void execute() {
        /*
         * double strafeSpeed = strafeRateLimiter.calculate(
         * strafeController.calculate(horizontalOffset, 0));
         * double rotationRate = rotationRateLimiter.calculate(
         * rotationController.calculate(horizontalOffset, 0));
         */

        if (!vision.hasTarget()) {
            stopMovement();
            return;
        }

        int tagId = (int) vision.getTagId();
        if (tagId < 1 || tagId > 22) {
            stopMovement();
            return;
        }

        // Get vision measurements (in meters)
        double horizontalOffset = vision.getHorizontalOffset();

        // Scale gains down as we get closer to target
        double distanceScale = Math.abs(horizontalOffset) / 10.0; // Assuming max tx is ~10 degrees
        distanceScale = Math.max(0.3, distanceScale); // Never scale below 30%

        // Apply scaled gains
        strafeController.setP(Constraints.TRANSLATION_P * distanceScale);
        rotationController.setP(Constraints.ROTATION_P * distanceScale);

        // Calculate control outputs
        double strafeSpeed = strafeController.calculate(horizontalOffset, 0);
        double rotationRate = rotationController.calculate(horizontalOffset, 0);

        // Apply movement
        drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationRate));

        updateTelemetry(horizontalOffset, strafeSpeed, rotationRate);
    }

    private void stopMovement() {
        drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    private void updateTelemetry(double offset, double strafeSpeed, double rotationRate) {
        SmartDashboard.putNumber("Vision/HorizontalOffset", offset);
        SmartDashboard.putNumber("Vision/StrafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("Vision/RotationRate", rotationRate);
        SmartDashboard.putBoolean("Vision/AtTarget", isFinished());
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() ||
                vision.getState() == VisionState.TARGET_LOCKED ||
                (strafeController.atGoal() && rotationController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        strafeController.reset(new TrapezoidProfile.State());
        rotationController.reset(new TrapezoidProfile.State());

    }

}
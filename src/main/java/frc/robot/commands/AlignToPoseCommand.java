package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Improved command to align robot to target using vision and motion profiling.
 * Uses ProfiledPIDController for smoother motion control.
 */
public class AlignToPoseCommand extends Command {
        // Subsystem dependencies
        private final CommandSwerveDrivetrain drivetrain;
        private final VisionSubsystem vision;

        // Motion constraints for profiled PID controllers
        private static final TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        VisionConstants.MAX_TRANSLATION_VELOCITY, // Max velocity in m/s
                        VisionConstants.MAX_TRANSLATION_ACCELERATION // Max acceleration in m/s^2
        );

        private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                        VisionConstants.MAX_ROTATION_VELOCITY, // Max velocity in rad/s
                        VisionConstants.MAX_ROTATION_ACCELERATION // Max acceleration in rad/s^2
        );

        // PID Controllers with motion profiling
        private final ProfiledPIDController xController;
        private final ProfiledPIDController yController;
        private final ProfiledPIDController rotationController;

        // Swerve drive request
        private final SwerveRequest.RobotCentric drive;

        // State tracking
        private final Pose2d targetPose;
        private double lastDistance = 0.0;
        private double lastHorizontalOffset = 0.0;

        public AlignToPoseCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
                this.vision = vision;
                this.drivetrain = drivetrain;
                this.targetPose = targetPose;

                // Initialize controllers with motion profiling
                xController = new ProfiledPIDController(
                                VisionConstants.TRANSLATION_kP,
                                VisionConstants.TRANSLATION_kI,
                                VisionConstants.TRANSLATION_kD,
                                TRANSLATION_CONSTRAINTS,
                                0.02 // 50Hz update rate
                );

                yController = new ProfiledPIDController(
                                VisionConstants.TRANSLATION_kP,
                                VisionConstants.TRANSLATION_kI,
                                VisionConstants.TRANSLATION_kD,
                                TRANSLATION_CONSTRAINTS,
                                0.02);

                rotationController = new ProfiledPIDController(
                                VisionConstants.ROTATION_kP,
                                VisionConstants.ROTATION_kI,
                                VisionConstants.ROTATION_kD,
                                ROTATION_CONSTRAINTS,
                                0.02);

                // Configure controllers
                configurePIDControllers();

                // Initialize drive request
                drive = new SwerveRequest.RobotCentric();

                // Require subsystems
                addRequirements(vision, drivetrain);
        }

        private void configurePIDControllers() {
                // Set tolerances
                xController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE_METERS);
                yController.setTolerance(VisionConstants.TRANSLATION_TOLERANCE_METERS);
                rotationController.setTolerance(VisionConstants.ROTATION_TOLERANCE_RADIANS);

                // Enable continuous input for rotation (-π to π)
                rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void initialize() {
                // Get current state
                double currentDistance = vision.getVerticalOffset();
                double currentHorizontalOffset = vision.getHorizontalOffset();
                var currentState = drivetrain.getState();

                xController.reset(
                                currentDistance,
                                currentState.Speeds.vxMetersPerSecond);

                yController.reset(
                                currentHorizontalOffset,
                                currentState.Speeds.vyMetersPerSecond);

                rotationController.reset(
                                // currentPose.getRotation().getRadians(),
                                currentState.Pose.getRotation().getRadians(),
                                currentState.Speeds.omegaRadiansPerSecond);

                // Store initial states
                lastDistance = currentDistance;
                lastHorizontalOffset = currentHorizontalOffset;

                // Log initial state
                updateTelemetry();
        }

        @Override
        public void execute() {
                if (!vision.hasTarget()) {
                        stopMovement();
                        updateTelemetry();
                        return;
                }

                // Get current measurements
                double currentDistance = vision.getVerticalOffset();
                double currentHorizontalOffset = vision.getHorizontalOffset();
                var robotPose = drivetrain.getState().Pose;

                // Calculate motion-profiled speeds
                double xSpeed = xController.calculate(
                                currentDistance,
                                VisionConstants.TARGET_DISTANCE_METERS);

                double ySpeed = yController.calculate(
                                currentHorizontalOffset,
                                0.0 // Target centered horizontally
                );

                double rotationSpeed = rotationController.calculate(
                                robotPose.getRotation().getRadians(),
                                targetPose.getRotation().getRadians());

                // Apply movement with direction compensation for Limelight mounting
                drivetrain.setControl(drive
                                .withVelocityX(xSpeed * VisionConstants.LIMELIGHT_DIRECTION)
                                .withVelocityY(ySpeed)
                                .withRotationalRate(rotationSpeed));

                // Update state tracking
                lastDistance = currentDistance;
                lastHorizontalOffset = currentHorizontalOffset;

                // Update telemetry
                updateTelemetry();
        }

        @Override
        public boolean isFinished() {
                return !vision.hasTarget() || (xController.atGoal() &&
                                yController.atGoal() &&
                                rotationController.atGoal());
        }

        @Override
        public void end(boolean interrupted) {
                stopMovement();
                SmartDashboard.putBoolean("Vision/AlignmentActive", false);
        }

        private void stopMovement() {
                drivetrain.setControl(drive
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0));
        }

        private void updateTelemetry() {
                // Controller states
                SmartDashboard.putNumber("Vision/Distance/Error", xController.getPositionError());
                SmartDashboard.putNumber("Vision/Horizontal/Error", yController.getPositionError());
                SmartDashboard.putNumber("Vision/Rotation/Error", rotationController.getPositionError());

                // Target tracking
                SmartDashboard.putBoolean("Vision/HasTarget", vision.hasTarget());
                SmartDashboard.putNumber("Vision/CurrentDistance", vision.getVerticalOffset());
                SmartDashboard.putNumber("Vision/HorizontalOffset", vision.getHorizontalOffset());

                // Controller goals
                SmartDashboard.putBoolean("Vision/AtXGoal", xController.atGoal());
                SmartDashboard.putBoolean("Vision/AtYGoal", yController.atGoal());
                SmartDashboard.putBoolean("Vision/AtRotationGoal", rotationController.atGoal());
        }
}
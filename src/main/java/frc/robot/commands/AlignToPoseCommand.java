package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

 // Tuning Chat https://claude.ai/chat/14464343-a583-4909-8d9d-35a1b91c53bd
 
public class AlignToPoseCommand extends Command {
        private final ShuffleboardTab tab = Shuffleboard.getTab("Vision Alignment");
        private final GenericEntry pGain = tab.add("Distance P Gain", VisionConstants.TRANSLATION_kP).getEntry();
        private final GenericEntry iGain = tab.add("Distance I Gain", VisionConstants.TRANSLATION_kI).getEntry();
        private final GenericEntry dGain = tab.add("Distance D Gain", VisionConstants.TRANSLATION_kD).getEntry();

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

                xController.setPID(
                        pGain.getDouble(VisionConstants.TRANSLATION_kP),
                        iGain.getDouble(VisionConstants.TRANSLATION_kI),
                        dGain.getDouble(VisionConstants.TRANSLATION_kD));

                // xController.reset();
                // yController.reset();
                // rotationController.reset();

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

                // https://claude.ai/chat/87d24d0c-551f-48b4-ae86-8ddc011ab7db
                // Get current measurements
                // FIXME Added LIMELIGHT_DIRECTION to correct for mounting direction
                double currentDistance = vision.getVerticalOffset() * VisionConstants.LIMELIGHT_DIRECTION;
                // FIXME Added LIMELIGHT_DIRECTION to correct for mounting direction
                double currentHorizontalOffset = vision.getHorizontalOffset() * VisionConstants.LIMELIGHT_DIRECTION;
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
                // FIXME: This is a temporary fix for the mounting direction of the Limelight
                drivetrain.setControl(drive
                                .withVelocityX(xSpeed * VisionConstants.LIMELIGHT_DIRECTION)
                                .withVelocityY(ySpeed * VisionConstants.LIMELIGHT_DIRECTION)
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
                // Add these new values
                SmartDashboard.putNumber("Pose VerticalOffset", vision.getVerticalOffset());
                SmartDashboard.putNumber("Pose HorizontalOffset", vision.getHorizontalOffset());
                SmartDashboard.putNumber("Pose Compensated/VerticalOffset",
                                vision.getVerticalOffset() * VisionConstants.LIMELIGHT_DIRECTION);
                SmartDashboard.putNumber("Pose Compensated/HorizontalOffset",
                                vision.getHorizontalOffset() * VisionConstants.LIMELIGHT_DIRECTION);
                SmartDashboard.putNumber("Pose LimelightDirection", VisionConstants.LIMELIGHT_DIRECTION);
                SmartDashboard.putBoolean("Pose LimelightFrontMounted", VisionConstants.LIMELIGHT_MOUNTED_ON_FRONT);

                // Controller states
                SmartDashboard.putNumber("Pose Distance/Error", xController.getPositionError());
                SmartDashboard.putNumber("Pose Horizontal/Error", yController.getPositionError());
                SmartDashboard.putNumber("Pose Rotation/Error", rotationController.getPositionError());

                // Target tracking
                SmartDashboard.putBoolean("Pose HasTarget", vision.hasTarget());
                SmartDashboard.putNumber("Pose CurrentDistance", vision.getVerticalOffset());
                SmartDashboard.putNumber("Pose HorizontalOffset", vision.getHorizontalOffset());

                // Controller goals
                SmartDashboard.putBoolean("Pose AtXGoal", xController.atGoal());
                SmartDashboard.putBoolean("Pose AtYGoal", yController.atGoal());
                SmartDashboard.putBoolean("Pose AtRotationGoal", rotationController.atGoal());
        }
}
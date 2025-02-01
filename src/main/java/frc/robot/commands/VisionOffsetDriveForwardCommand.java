package frc.robot.commands;

/*
 * Reference
 * https://claude.ai/chat/73d30e8e-1c9a-4a0e-9f04-0eca780cae63 
 */

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionOffsetDriveForwardCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;
    private final double targetOffset; // Added offset parameter

    // CTRE Swerve request remains the same
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController rotationController;

    // Modified constructor to accept offset
    public VisionOffsetDriveForwardCommand(
            VisionSubsystem vision,
            CommandSwerveDrivetrain drivetrain,
            CommandXboxController driverController,
            double targetOffset) { // Added parameter
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.targetOffset = targetOffset; // Store the offset

        // Only require vision subsystem since we want to blend driver input
        addRequirements(vision);

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(
                VisionConstants.MAX_TRANSLATIONAL_VELOCITY,
                VisionConstants.MAX_TRANSLATIONAL_ACCELERATION);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                VisionConstants.MAX_ANGULAR_VELOCITY, VisionConstants.MAX_ANGULAR_ACCELERATION);

        strafeController = new ProfiledPIDController(
                VisionConstants.TRANSLATIONAL_kP,
                VisionConstants.TRANSLATIONAL_kI,
                VisionConstants.TRANSLATIONAL_kD,
                translationConstraints,
                0.02);

        rotationController = new ProfiledPIDController(
                VisionConstants.ANGULAR_kP,
                VisionConstants.ANGULAR_kI,
                VisionConstants.ANGULAR_kD,
                rotationConstraints,
                0.02);

        configurePIDControllers();
        addRequirements(vision, drivetrain);

    }

    private void configurePIDControllers() {
        strafeController.setTolerance(VisionConstants.POSITION_TOLERANCE);
        rotationController.setTolerance(VisionConstants.ANGULAR_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        strafeController.reset(0, 0);
        rotationController.reset(0, 0);
    }


    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            return;
        }

        // Calculate vision alignment velocities with offset
        double horizontalOffset = vision.getHorizontalOffset() - targetOffset; // Apply offset
        double strafeSpeed = strafeController.calculate(horizontalOffset, 0);
        double rotationRate = calculateRotationRate();

        // Blend with driver input
        double driverForward = -driverController.getLeftY() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double driverStrafe = -driverController.getLeftX() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double driverRotation = -driverController.getRightX() * VisionConstants.MAX_ANGULAR_VELOCITY;

        // Combine vision and driver inputs
        drivetrain.setControl(drive
                .withVelocityX(driverForward)
                .withVelocityY(strafeSpeed + driverStrafe)
                .withRotationalRate(rotationRate + driverRotation));

        updateTelemetry(horizontalOffset, strafeSpeed, rotationRate);
    }

    private double calculateStrafeSpeed() {
        double horizontalOffset = vision.getHorizontalOffset();
        return strafeController.calculate(horizontalOffset, 0);
    }

    private double calculateRotationRate() {
        double horizontalOffset = vision.getHorizontalOffset();
        return rotationController.calculate(Units.degreesToRadians(horizontalOffset), 0);
    }

    private void updateTelemetry(double offset, double strafeSpeed, double rotationRate) {
        SmartDashboard.putNumber("V5l/HorizontalOffset", offset);
        SmartDashboard.putNumber("V5l/StrafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("V5l/RotationRate", rotationRate);
        SmartDashboard.putNumber("V5l/StrateError", strafeController.getPositionError());
        SmartDashboard.putNumber("V5l/RotationError", rotationController.getPositionError());
        SmartDashboard.putBoolean("V5l/AtTarget", isFinished());
        SmartDashboard.putString("V5l/DriveRequestType", drive.DriveRequestType.toString());
        SmartDashboard.putString("V5l/SteerRequestType", drive.SteerRequestType.toString());
        SignalLogger.writeDouble("V5l/HorizontalOffset", offset);
        SignalLogger.writeDouble("V5l/StrafeSpeed", strafeSpeed);
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() ||
                vision.getState() == VisionState.TARGET_LOCKED ||
                (strafeController.atGoal() && rotationController.atGoal());
    }

    private void stopMovement() {
        drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        // strafeController.reset();
        // rotationController.reset();
        strafeController.reset(new TrapezoidProfile.State(0, 0));
        rotationController.reset(new TrapezoidProfile.State(0, 0));
    }

}
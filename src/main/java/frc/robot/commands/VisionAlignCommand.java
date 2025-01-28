package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class VisionAlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final int targetTagId;
    private final Timer timeoutTimer = new Timer();
    private final double timeoutSeconds;
    
    // PID Controllers for alignment
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    // CTRE SwerveRequest for driving
    private final SwerveRequest.FieldCentric driveRequest;

    // Tunable constants 
    private static final double X_P = 0.7;
    private static final double X_I = 0.0;
    private static final double X_D = 0.02;
    private static final double Y_P = 0.7; 
    private static final double Y_I = 0.0;
    private static final double Y_D = 0.02;
    private static final double ROT_P = 0.03;
    private static final double ROT_I = 0.0;
    private static final double ROT_D = 0.001;

    // Maximum speeds
    private static final double MAX_DRIVE_SPEED = 3.0; // meters/second
    private static final double MAX_ROTATE_SPEED = 1.5; // radians/second

    // Tolerance values
    private static final double DISTANCE_TOLERANCE_METERS = 0.05;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;

    public VisionAlignCommand(CommandSwerveDrivetrain drivetrain, int targetTagId, double timeoutSeconds) {
        this.drivetrain = drivetrain;
        this.targetTagId = targetTagId;
        this.timeoutSeconds = timeoutSeconds;

        // Initialize PID controllers
        xController = new PIDController(X_P, X_I, X_D);
        yController = new PIDController(Y_P, Y_I, Y_D);
        rotationController = new PIDController(ROT_P, ROT_I, ROT_D);

        // Set tolerances
        xController.setTolerance(DISTANCE_TOLERANCE_METERS);
        yController.setTolerance(DISTANCE_TOLERANCE_METERS);
        rotationController.setTolerance(ANGLE_TOLERANCE_DEGREES);

        // Make rotation controller continuous
        rotationController.enableContinuousInput(-180, 180);

        // Initialize CTRE drive request
        driveRequest = new SwerveRequest.FieldCentric();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timeoutTimer.restart();
        
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    @Override 
    public void execute() {
        // Check if we see our target tag
        if (LimelightHelpers.getFiducialID("limelight") == targetTagId) {
            // Get target pose data from Limelight
            double tx = LimelightHelpers.getTX("limelight");
            double ty = LimelightHelpers.getTY("limelight");
            double ta = LimelightHelpers.getTA("limelight");

            // Calculate drive outputs using PID
            double xSpeed = xController.calculate(ty, 0);
            double ySpeed = yController.calculate(tx, 0);
            double rotationSpeed = rotationController.calculate(tx, 0);

            // Limit speeds
            xSpeed = Math.min(Math.max(xSpeed, -MAX_DRIVE_SPEED*.25), MAX_DRIVE_SPEED*.25);
            ySpeed = Math.min(Math.max(ySpeed, -MAX_DRIVE_SPEED*.25), MAX_DRIVE_SPEED*.25);
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATE_SPEED*.25), MAX_ROTATE_SPEED*.25);

            // Apply deadband
            xSpeed = applyDeadband(xSpeed, 0.05);
            ySpeed = applyDeadband(ySpeed, 0.05);
            rotationSpeed = applyDeadband(rotationSpeed, 0.05);

            // Command the drivetrain using CTRE's FieldCentric request
            drivetrain.setControl(driveRequest
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
        } else {
            // If we don't see the target, stop
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        // Check if we've timed out
        if (timeoutTimer.hasElapsed(timeoutSeconds)) {
            return true;
        }

        // Check if we're at our target and see the correct tag
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint() &&
               LimelightHelpers.getFiducialID("limelight") == targetTagId;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain using CTRE's request
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }
}
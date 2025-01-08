package frc.robot.experimental;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/*
 * More complex command that aligns the robot's full pose (position and rotation) to a specified target pose
 */

public class AlignToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    private final SwerveRequest.RobotCentric drive;
    private VisionSubsystem vision;

    // PID constants - tune these for your robot
    private static final double TRANSLATION_P = 1.0;
    private static final double TRANSLATION_I = 0.0;
    private static final double TRANSLATION_D = 0.0;
    
    private static final double ROTATION_P = 0.05;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;

    // Tolerance constants
    private static final double TRANSLATION_TOLERANCE = 0.02; // meters
    private static final double ROTATION_TOLERANCE = 2.0; // degrees

    public AlignToPoseCommand(frc.robot.subsystems.vision.VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        
        xController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        yController = new PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D);
        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        
        // Set tolerances
        xController.setTolerance(TRANSLATION_TOLERANCE);
        yController.setTolerance(TRANSLATION_TOLERANCE);
        rotationController.setTolerance(ROTATION_TOLERANCE);
        
        // Make rotation controller continuous
        rotationController.enableContinuousInput(-180, 180);
        
        drive = new SwerveRequest.RobotCentric();

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Calculate required movements
        double xSpeed = xController.calculate(
            currentPose.getX(),
            targetPose.getX()
        );
        
        double ySpeed = yController.calculate(
            currentPose.getY(),
            targetPose.getY()
        );
        
        double rotationSpeed = rotationController.calculate(
            currentPose.getRotation().getDegrees(),
            targetPose.getRotation().getDegrees()
        );
        
        // Apply speeds to swerve drive
        drivetrain.setControl(drive
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationSpeed));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() &&
               yController.atSetpoint() &&
               rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }
}
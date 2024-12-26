package frc.robot.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class AlignToTargetCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotationController;
    private final PIDController forwardController;
    
    private final SwerveRequest.RobotCentric drive;

    // PID constants - tune these for your robot
    private static final double ROTATION_P = 0.035;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.001;
    
    private static final double FORWARD_P = 0.1;
    private static final double FORWARD_I = 0.0;
    private static final double FORWARD_D = 0.0;

    // Target area setpoint - tune this for your desired distance
    private static final double TARGET_AREA = 2.0;

    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotationController.setTolerance(1.0); // Degrees
        
        forwardController = new PIDController(FORWARD_P, FORWARD_I, FORWARD_D);
        forwardController.setTolerance(0.1); // Area units
        
        drive = new SwerveRequest.RobotCentric();

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        rotationController.reset();
        forwardController.reset();
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            // Calculate rotation speed to center target
            double rotationSpeed = rotationController.calculate(vision.getTargetXOffset(), 0);
            
            // Calculate forward speed to maintain distance
            double forwardSpeed = -forwardController.calculate(vision.getTargetArea(), TARGET_AREA);
            
            // Apply speeds to swerve drive
            drivetrain.setControl(drive
                .withVelocityX(forwardSpeed)
                .withVelocityY(0.0)
                .withRotationalRate(rotationSpeed));
        } else {
            // No target - stop moving
            drivetrain.setControl(drive
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        }
    }

    @Override
    public boolean isFinished() {
        if (!vision.hasValidTarget()) {
            return false;
        }
        // Command finishes when robot is aligned and at correct distance
        return rotationController.atSetpoint() && forwardController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving when command ends
        drivetrain.setControl(drive
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }
}
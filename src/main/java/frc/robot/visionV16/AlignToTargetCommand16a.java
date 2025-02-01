package frc.robot.visionV16;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;

/*
 * The code as shown is using the Target Area (ta) value from the Limelight to approximate distance, but this isn't the most accurate method. 
 * Let me explain a better approach using Limelight's ty value and some trigonometry.
 * Here's how to calculate the actual distance to a target using the Limelight's vertical angle (ty):
 */

public class AlignToTargetCommand16a extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem16 vision;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    private static final double TARGET_DISTANCE = 1.0; // meters
    
    public AlignToTargetCommand16a(CommandSwerveDrivetrain drivetrain, VisionSubsystem16 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // Configure PID controllers
        xController = new PIDController(0.5, 0, 0);
        yController = new PIDController(0.5, 0, 0);
        rotationController = new PIDController(0.03, 0, 0);
        
        // Make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        vision.setLeds(true);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Calculate distance and angles
            double tx = vision.getTX();
            // double ty = vision.getTY();
            
            // Calculate control outputs
            double xSpeed = -xController.calculate(vision.getTA(), TARGET_DISTANCE);
            double ySpeed = -yController.calculate(tx, 0);
            double rotationSpeed = -rotationController.calculate(tx * Math.PI / 180.0, 0);
            
            // Apply to drivetrain
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
        }
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLeds(false);
        drivetrain.setControl(new SwerveRequest.FieldCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return vision.isAlignedToTarget() && Math.abs(vision.getTA() - TARGET_DISTANCE) < 0.1;
    }
}


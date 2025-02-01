package frc.robot.visionV16;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;


public class AlignToTargetCommand16b extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem16 vision;
    
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // private static final double TARGET_DISTANCE = 1.0; // meters
    
    public AlignToTargetCommand16b(CommandSwerveDrivetrain drivetrain, VisionSubsystem16 vision) {
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

    // Define your target distance based on game requirements
    private static final double TARGET_DISTANCE_METERS = 0.3; // Adjust based on your strategy
    
    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double currentDistance = vision.getDistanceToTargetMeters();
            
            // Calculate control outputs using real distance
            double xSpeed = -xController.calculate(currentDistance, TARGET_DISTANCE_METERS);
            double ySpeed = -yController.calculate(vision.getTX(), 0);
            double rotationSpeed = -rotationController.calculate(vision.getTX() * Math.PI / 180.0, 0);
            
            // Apply to drivetrain
            drivetrain.setControl(new SwerveRequest.FieldCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return vision.isAlignedToTarget() && 
               Math.abs(vision.getDistanceToTargetMeters() - TARGET_DISTANCE_METERS) < 0.1;
    }
}
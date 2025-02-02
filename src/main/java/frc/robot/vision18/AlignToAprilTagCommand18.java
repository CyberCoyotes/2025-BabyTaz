package frc.robot.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToAprilTagCommand18 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // Add SwerveRequest for robot-centric drive
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    
    public AlignToAprilTagCommand18(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        xController = new PIDController(0.5, 0.0, 0.02);
        yController = new PIDController(0.5, 0.0, 0.02);
        rotationController = new PIDController(1.0, 0.0, 0.05);
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX(vision.getName());
        double ty = LimelightHelpers.getTY(vision.getName());
        
        if (LimelightHelpers.getTV(vision.getName())) {
            // Calculate drive outputs
            double xSpeed = xController.calculate(ty, 0); // Changed for back mounted limelight
            double ySpeed = -yController.calculate(tx, 0);
            double rotationSpeed = -rotationController.calculate(
                drivetrain.getState().Pose.getRotation().getRadians(), 0);
            
            // Command drivetrain using robot-centric request
            drivetrain.setControl(robotCentric
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed) 
                .withRotationalRate(rotationSpeed));
        } else {
            drivetrain.stopDrive();
        }
    }
    
    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint();
    }
}
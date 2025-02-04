package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTagCommand18x extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    
    // Adjust PID controllers with lower gains and deadbands
    private final PIDController xController; // Controls forward/backward
    private final PIDController yController; // Controls left/right
    private final PIDController rotationController; // Controls rotation
    

    
    // Add SwerveRequest for robot-centric drive
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public AlignToTagCommand18x(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        xController = new PIDController(0.3, 0.0, 0.0);  // Distance control
        yController = new PIDController(0.4, 0.0, 0.0);  // Lateral control 
        rotationController = new PIDController(0.3, 0.0, 0.0); // Rotation control

        // Increase tolerance for testing
        xController.setTolerance(0.05);  // 5cm
        yController.setTolerance(0.05);  // 5cm
        rotationController.setTolerance(Math.toRadians(2.0));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            drivetrain.stopDrive();
            return;
        }
    
        // Get Limelight measurements
        double tx = LimelightHelpers.getTX(vision.getName());
        double ty = LimelightHelpers.getTY(vision.getName());
        
        // Calculate current distance 
        double currentDistance = calculateDistance(ty);
    
        // For X control (forward/back):
        // If we're too far, we need to move forward (positive X)
        double xSpeed = xController.calculate(currentDistance, VisionConstants18.TARGET_DISTANCE_METERS);
    
        // For Y control (left/right):
        // If target is to the right (positive tx), we need to move right (positive Y)
        double ySpeed = yController.calculate(tx, 0);
        
        // Apply speed limits
        double maxSpeed = 0.3;
        xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
        ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);
    
        // Send controls to drivetrain - note the correct mapping:
        drivetrain.setControl(robotCentric
            .withVelocityX(xSpeed)  // Forward/back based on distance error
            .withVelocityY(ySpeed)  // Left/right based on tx
            .withRotationalRate(0)); // Keep rotation fixed for now

            // Add these in execute():
            SmartDashboard.putNumber("V18x/Current_Distance", currentDistance);
            SmartDashboard.putNumber("V18x/Target_Distance", VisionConstants18.TARGET_DISTANCE_METERS);
            SmartDashboard.putNumber("V18x/Distance_Error", VisionConstants18.TARGET_DISTANCE_METERS - currentDistance);
            SmartDashboard.putNumber("V18x/X_Speed", xSpeed);
    }
    
    //
    @Override
    public boolean isFinished() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            return false;
        }
        
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }

     // Modified distance calculation for accuracy
     private double calculateDistance(double ty) {
        double limelightHeightMeters = Units.inchesToMeters(12); // Verify this height
        double limelightMountAngleDegrees = 0.0; // Updated for front mount
        double targetHeightMeters = Units.inchesToMeters(7); // Verify this target height
        
        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
        return (targetHeightMeters - limelightHeightMeters) / Math.tan(angleToGoalRadians);
    }
}

/*
Monday night adjustments
https://claude.ai/chat/cbf2369b-de90-4674-8839-5b0d91dab2a2

Based on your description and the code shown, there are several issues to address:

First, there's a duplicate control call in AlignToAprilTagCommand18.execute() - you're setting the control twice which could cause issues.
The robot maintaining a steady TY while TX becomes more negative suggests the distance control (X axis) isn't responding properly and the robot is moving in an arc pattern instead of correcting its distance.
 */

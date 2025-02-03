package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlignToAprilTagCommand18 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    
    // Adjust PID controllers with lower gains and deadbands
    private final PIDController xController; // Controls forward/backward
    private final PIDController yController; // Controls left/right
    private final PIDController rotationController; // Controls rotation
    

    
    // Add SwerveRequest for robot-centric drive
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public AlignToAprilTagCommand18(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Reduce gains and add integral term for steady-state error
        xController = new PIDController(0.2, 0.00, 0.00);
        yController = new PIDController(0.2, 0.00, 0.00);
        rotationController = new PIDController(0.3, 0.00, 0.00);
        // PhoenixPIDController(0.7, 0.01, 0.02);


        // Configure controllers
        xController.setTolerance(VisionConstants18.DEADBAND_METERS);
        yController.setTolerance(VisionConstants18.DEADBAND_METERS);
        rotationController.setTolerance(VisionConstants18.ROTATION_DEADBAND_DEGREES);

        // Keep rotation between -pi and pi
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
        
        // Calculate distance (using your existing method)
        double currentDistance = calculateDistance(ty);
        
        // Calculate drive outputs with signs adjusted for back-mounted camera
        double xSpeed = xController.calculate(currentDistance, VisionConstants18.TARGET_DISTANCE_METERS);
        double ySpeed = yController.calculate(tx, 0);
        
        // Calculate rotation to face the target
        // Note: Adjust sign if rotation direction is incorrect
        double rotationSpeed = rotationController.calculate(
            drivetrain.getState().Pose.getRotation().getRadians(), 0);

        // Apply stricter speed limits
        double maxSpeed = 0.3; // 30% max speed
        xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
        ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxSpeed, maxSpeed);

        // Since camera is rear-mounted, we need to invert some controls
        drivetrain.setControl(robotCentric
            .withVelocityX(xSpeed)  // Invert X because camera is on back
            .withVelocityY(ySpeed)  // Invert Y to match camera perspective
            .withRotationalRate(rotationSpeed));
            
            // Log values for debugging
            SmartDashboard.putNumber("V18/Distance", currentDistance);
            SmartDashboard.putNumber("V18/TX", tx);
            SmartDashboard.putNumber("V18/TY", ty);
            // SmartDashboard.putNumber("V18/TA", ta);
            SmartDashboard.putNumber("V18/XSpeed", xSpeed);
            SmartDashboard.putNumber("V18/YSpeed", ySpeed);
            SmartDashboard.putNumber("V18/RotSpeed", rotationSpeed);
            
            // Command drivetrain using robot-centric request
            drivetrain.setControl(robotCentric
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));
        /*
        } else {
            drivetrain.stopDrive();
        }
        */
    }

    // Helper method to calculate distance using ty
    private double calculateDistance(double ty) {
        // Use your Limelight mounting height and angle to calculate distance
        double limelightHeightMeters = 0.5; // Adjust to your mounting height
        double limelightMountAngleDegrees = 30.0; // Adjust to your mount angle
        double targetHeightMeters = 0.6; // Adjust to AprilTag height
        
        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
        return (targetHeightMeters - limelightHeightMeters) / Math.tan(angleToGoalRadians);
    }

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
}
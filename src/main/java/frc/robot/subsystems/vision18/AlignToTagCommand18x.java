package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToTagCommand18x extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    
    // Only using X controller for distance testing
    private final PIDController xController;
    
    // Robot-centric drive for testing
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public AlignToTagCommand18x(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Start with lower gain for testing
        xController = new PIDController(0.2, 0.0, 0.0);  // Reduced from 0.3
        
        // Larger tolerance for initial testing
        xController.setTolerance(0.10);  // 10cm tolerance
        
        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
        // Verify we have a target
        if (!LimelightHelpers.getTV(vision.getName())) {
            SmartDashboard.putBoolean("XTest/HasTarget", false);
            drivetrain.stopDrive();
            return;
        }
        SmartDashboard.putBoolean("XTest/HasTarget", true);

        // Get Limelight measurements
        double ty = LimelightHelpers.getTY(vision.getName());
        
        // Calculate and log current distance
        double currentDistance = calculateDistance(ty);
        double targetDistance = VisionConstants18.TARGET_DISTANCE_METERS;
        double distanceError = targetDistance - currentDistance;
        
        // Calculate X speed for forward/back movement
        double xSpeed = xController.calculate(currentDistance, targetDistance);
        
        // Apply speed limit
        double maxSpeed = 0.2; // Reduced for testing
        xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);

        // Log all values for tuning
        SmartDashboard.putNumber("XTest/TY", ty);
        SmartDashboard.putNumber("XTest/CurrentDistance", currentDistance);
        SmartDashboard.putNumber("XTest/TargetDistance", targetDistance);
        SmartDashboard.putNumber("XTest/DistanceError", distanceError);
        SmartDashboard.putNumber("XTest/XSpeed", xSpeed);
        SmartDashboard.putBoolean("XTest/AtSetpoint", xController.atSetpoint());
        
        // Send control - only X movement, no Y or rotation
        drivetrain.setControl(robotCentric
            .withVelocityX(xSpeed)  // Forward/back only
            .withVelocityY(0)       // No left/right
            .withRotationalRate(0)); // No rotation
    }
    
    @Override
    public boolean isFinished() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            return false;
        }
        
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
        SmartDashboard.putBoolean("XTest/CommandActive", false);
    }

    // Distance calculation with detailed comments
    private double calculateDistance(double ty) {
        // FIXME: Update these measurements for your robot!
        double limelightHeightMeters = 0.5;  // Height of Limelight lens from floor
        double limelightMountAngleDegrees = 0.0; // Angle of Limelight from horizontal
        double targetHeightMeters = 0.6; // Height of AprilTag center from floor
        
        // Convert ty (vertical angle to target) to radians and add mount angle
        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
        
        // Calculate distance using trigonometry
        // distance = opposite / tan(angle)
        return (targetHeightMeters - limelightHeightMeters) / Math.tan(angleToGoalRadians);
    }
}
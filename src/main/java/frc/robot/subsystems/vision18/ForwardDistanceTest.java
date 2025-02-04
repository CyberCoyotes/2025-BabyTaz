package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ForwardDistanceTest extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    private final PIDController distanceController;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    
    // Increased target distance based on testing
    private static final double TARGET_DISTANCE = 2.0; // meters

    public ForwardDistanceTest(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // Remove the negative since we want:
        // positive error (too far) = positive speed (move forward)
        // negative error (too close) = negative speed (move back)
        distanceController = new PIDController(0.15, 0, 0);
        distanceController.setTolerance(0.1); // 10cm tolerance
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            drivetrain.stopDrive();
            return;
        }

        // Get current distance
        double currentDistance = calculateDistance(LimelightHelpers.getTY(vision.getName()));
        
        // Calculate error (positive means we need to move forward)
        double distanceError = TARGET_DISTANCE - currentDistance;
        
        // Calculate speed (removed negative)
        double forwardSpeed = distanceController.calculate(currentDistance, TARGET_DISTANCE);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.2, 0.2); // Limit speed for testing

        // Debug values
        SmartDashboard.putNumber("FwdTest/CurrentDistance", currentDistance);
        SmartDashboard.putNumber("FwdTest/TargetDistance", TARGET_DISTANCE);
        SmartDashboard.putNumber("FwdTest/DistanceError", distanceError);
        SmartDashboard.putNumber("FwdTest/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("FwdTest/TY", LimelightHelpers.getTY(vision.getName()));

        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed) // Positive = forward
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private double calculateDistance(double ty) {
        // 2024 AprilTag mounting heights
        double cameraHeight = 0.5;      // FIXME: Measure on robot
        double targetHeight = 1.45;     // Center of AprilTag
        double cameraAngle = 0;         // FIXME: Measure on robot
        
        double angleToTarget = cameraAngle + ty;
        SmartDashboard.putNumber("FwdTest/AngleToTarget", angleToTarget);
        
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angleToTarget));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }
}
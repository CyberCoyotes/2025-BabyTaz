package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ForwardDistanceTest extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    private final PIDController distanceController;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    
    // Target distance - adjust as needed
    private static final double TARGET_DISTANCE = 1.0; // Needs to be 0.5 to 1.5 meters because space constraints

    public ForwardDistanceTest(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // Configure PID
        distanceController = new PIDController(0.15, 0, 0);
        distanceController.setTolerance(0.1); // 10cm tolerance
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            SmartDashboard.putString("FwdTest/Status", "NO TARGET");
            drivetrain.stopDrive();
            return;
        }

        // Get current distance
        double currentDistance = calculateDistance(LimelightHelpers.getTY(vision.getName()));
        
        // Reverse our distance calculation logic
        double forwardSpeed = -distanceController.calculate(currentDistance, TARGET_DISTANCE);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -0.15, 0.15); // Reduced max speed

        // Very explicit debug info
        String direction = forwardSpeed > 0 ? "MOVING FORWARD" : "MOVING BACKWARD";
        if (Math.abs(forwardSpeed) < 0.01) direction = "STOPPED";
        
        SmartDashboard.putString("FwdTest/Status", direction);
        SmartDashboard.putString("FwdTest/Distance Status", 
            currentDistance > TARGET_DISTANCE ? "TOO FAR" : "TOO CLOSE");
        SmartDashboard.putNumber("FwdTest/Current Distance (m)", currentDistance);
        SmartDashboard.putNumber("FwdTest/Target Distance (m)", TARGET_DISTANCE);
        SmartDashboard.putNumber("FwdTest/TY Value", LimelightHelpers.getTY(vision.getName()));
        SmartDashboard.putNumber("FwdTest/Speed", forwardSpeed);

        // Apply control
        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private double calculateDistance(double ty) {
        // Use your custom 2025 game measurements here
        double cameraHeight = Units.inchesToMeters(12.5);      // COnfirmed: Update with your height
        double targetHeight = Units.inchesToMeters(25.5);      // Confirmed: Update with your target height
        double cameraAngle = 0;         // Confirmed: Update with your mount angle
        
        double angleToTarget = cameraAngle + ty;
        return (targetHeight - cameraHeight) / Math.tan(Math.toRadians(angleToTarget));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
        SmartDashboard.putString("FwdTest/Status", "COMMAND ENDED");
    }
}
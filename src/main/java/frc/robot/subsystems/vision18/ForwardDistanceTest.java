package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

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
        distanceController = new PIDController(
            1.0, /* 0.5 was good, 0.75 good */ 
            0, 
            0);
        distanceController.setTolerance(0.1); // 10cm tolerance
        addRequirements(drivetrain);

        // Set up AdvantageKit logging metadata
        Logger.recordMetadata("ForwardTest/Description", "Command for testing forward/back distance control");
        Logger.recordMetadata("ForwardTest/TargetDistance", String.format("%.2f meters", TARGET_DISTANCE));
        Logger.recordMetadata("ForwardTest/PID_Values", 
            String.format("kP: %.3f, kI: %.3f, kD: %.3f", 
                distanceController.getP(),
                distanceController.getI(), 
                distanceController.getD()));
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            SmartDashboard.putString("FwdTest/Status", "NO TARGET");
            drivetrain.stopDrive();
            return;
        }
    
        double ty = LimelightHelpers.getTY(vision.getName());
        double currentDistance = calculateDistance(ty);
        
        // Calculate error and speed
        double distanceError = currentDistance - TARGET_DISTANCE;
        double forwardSpeed = distanceController.calculate(currentDistance, TARGET_DISTANCE);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -1, 1);
    
        // Debug info
        SmartDashboard.putNumber("FwdTest/TY", ty);
        SmartDashboard.putNumber("FwdTest/Distance_Error", distanceError);
        SmartDashboard.putNumber("FwdTest/Current_Distance", currentDistance);
        SmartDashboard.putNumber("FwdTest/Target_Distance", TARGET_DISTANCE);
        SmartDashboard.putNumber("FwdTest/Forward_Speed", forwardSpeed);
        SmartDashboard.putString("FwdTest/Status", 
            String.format("Distance: %.2fm, Error: %.2fm, Speed: %.2f", 
                currentDistance, distanceError, forwardSpeed));
    
        // Log all relevant data
        Logger.recordOutput("ForwardTest/TY_Angle", ty);
        Logger.recordOutput("ForwardTest/Current_Distance", currentDistance);
        Logger.recordOutput("ForwardTest/Distance_Error", distanceError);
        Logger.recordOutput("ForwardTest/Forward_Speed", forwardSpeed);
        Logger.recordOutput("ForwardTest/At_Setpoint", distanceController.atSetpoint());
        
        // Log status as a string
        String status = String.format("D:%.2fm E:%.2fm S:%.2f", 
            currentDistance, distanceError, forwardSpeed);
        Logger.recordOutput("ForwardTest/Status", status);

        // Log PID calculation components
        Logger.recordOutput("ForwardTest/PID/Proportional", 
            distanceController.getP() * distanceError);
        Logger.recordOutput("ForwardTest/PID/Integral",
            distanceController.getI() * distanceController.getPositionError());
        Logger.recordOutput("ForwardTest/PID/Derivative",
            distanceController.getD() * distanceController.getVelocityError());

        // Apply control
        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    /* 
    * Calculate distance to target using Limelight TY value as an absolute value 
    */ 
    private double calculateDistance(double ty) {
        // Confirmed measurements
        double cameraHeight = Units.inchesToMeters(12.5);  // 12.5 inches
        double targetHeight = Units.inchesToMeters(25.5);  // 25.5 inches
        double cameraAngle = 0;                            // 0 degrees
        
        double angleToTarget = cameraAngle + ty;
        
        // Add absolute value to ensure positive distance
        return Math.abs((targetHeight - cameraHeight) / Math.tan(Math.toRadians(angleToTarget)));
    }
    

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
        SmartDashboard.putString("FwdTest/Status", "COMMAND ENDED");
    }

    // Add method to set up AdvantageScope layout
    public static void setupAdvantagescopeLayout() {
        // Main tab layout
        Logger.recordMetadata("ForwardTest/TabName", "Forward Distance Test");
        
        // Line plots
        Logger.recordMetadata("ForwardTest/Current_Distance", "Line Plot");
        Logger.recordMetadata("ForwardTest/Distance_Error", "Line Plot");
        Logger.recordMetadata("ForwardTest/Forward_Speed", "Line Plot");
        Logger.recordMetadata("ForwardTest/TY_Angle", "Line Plot");
        
        // PID component plots
        Logger.recordMetadata("ForwardTest/PID/Proportional", "Line Plot");
        Logger.recordMetadata("ForwardTest/PID/Integral", "Line Plot");
        Logger.recordMetadata("ForwardTest/PID/Derivative", "Line Plot");
        
        // Status indicators
        Logger.recordMetadata("ForwardTest/HasTarget", "Boolean");
        Logger.recordMetadata("ForwardTest/At_Setpoint", "Boolean");
        Logger.recordMetadata("ForwardTest/Status", "String");
        Logger.recordMetadata("ForwardTest/Command_State", "String");
    }
}
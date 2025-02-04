package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
    
    // Only using X controller for distance testing
    private final PIDController forwardController;
    
    // Robot-centric drive for testing
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    // Constants for distance calculation
    private final double TARGET_DISTANCE = 1.0; // meters - adjust as needed
    private final double MAX_SPEED = 0.15; // Start with slow speed for testing

    public AlignToTagCommand18x(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // PID for forward/back motion
        forwardController = new PIDController(0.15, 0.0, 0.0);  // Lower P gain for testing
        forwardController.setTolerance(0.10);  // 10cm tolerance
        
        addRequirements(drivetrain);
    }

    @Override 
    public void execute() {
        // Debug if we see target
        boolean hasTarget = LimelightHelpers.getTV(vision.getName());
        SmartDashboard.putBoolean("FwdTest/HasTarget", hasTarget);
        
        if (!hasTarget) {
            drivetrain.stopDrive();
            return;
        }

        // Get vertical angle to target (ty)
        double targetY = LimelightHelpers.getTY(vision.getName());
        SmartDashboard.putNumber("FwdTest/TargetY", targetY);
        
        // Calculate current distance using ty
        double currentDistance = calculateDistance(targetY);
        double distanceError = TARGET_DISTANCE - currentDistance;
        
        // Calculate forward/back speed
        double forwardSpeed = forwardController.calculate(currentDistance, TARGET_DISTANCE);
        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_SPEED, MAX_SPEED);

        // Log everything for debugging
        SmartDashboard.putNumber("FwdTest/CurrentDistance", currentDistance);
        SmartDashboard.putNumber("FwdTest/TargetDistance", TARGET_DISTANCE);
        SmartDashboard.putNumber("FwdTest/DistanceError", distanceError);
        SmartDashboard.putNumber("FwdTest/ForwardSpeed", forwardSpeed);
        SmartDashboard.putBoolean("FwdTest/AtSetpoint", forwardController.atSetpoint());
        
        // ONLY move forward/back - explicitly set other axes to 0
        drivetrain.setControl(robotCentric
            .withVelocityX(forwardSpeed)  // Forward/back movement
            .withVelocityY(0.0)           // Force no left/right
            .withRotationalRate(0.0));     // Force no rotation
    }

    @Override
    public boolean isFinished() {
        return false; // Run until button is released for testing
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }

    private double calculateDistance(double ty) {
        // TODO: Update these with your actual measured values!
        double limelightHeightMeters = Units.inchesToMeters(12);  // Example: LL is 24" off ground
        double tagHeightMeters = Units.inchesToMeters(11); // 2024 AprilTag center height
        double mountAngleDegrees = 0.0; // Limelight mount angle from horizontal
        
        // Convert angles to radians
        double angleToTargetRad = Math.toRadians(mountAngleDegrees + ty);
        
        // Calculate distance using trigonometry
        double heightDifference = tagHeightMeters - limelightHeightMeters;
        return heightDifference / Math.tan(angleToTargetRad);
    }
}

/*
* https://claude.ai/chat/856872e3-4975-49be-adec-9fbeaadc81f4 
*/ 
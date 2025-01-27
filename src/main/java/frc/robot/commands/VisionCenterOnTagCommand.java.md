package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/*
 * This command will:

Use PID control for both rotation and strafing
Validate AprilTag IDs (1-22)
Allow live PID tuning via SmartDashboard
Provide telemetry for debugging
Use combined rotation and strafe movements to center on the target
Stop when either no target is visible or the robot is centered and squared up

You can tune the PID values and speed limits in the Gains class to match your robot's specific characteristics and requirements.

https://claude.ai/chat/71f069b3-88c6-40bf-851c-0a0e3dd93c98 
 */


// First, let's create the basic command structure with necessary dependencies:

public class VisionCenterOnTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Separate PID controllers for rotation and strafe
    private final PIDController rotationPID;
    private final PIDController strafePID;
    
    // Swerve drive request
    private final SwerveRequest.RobotCentric drive;
    
    // Constants for tuning
    private static final class Gains {
        // Rotation alignment gains
        static final double ROTATION_P = 0.1;
        static final double ROTATION_I = 0.0;
        static final double ROTATION_D = 0.01;
        // Strafe centering gains
        static final double STRAFE_P = 0.1;
        static final double STRAFE_I = 0.0;
        static final double STRAFE_D = 0.01;
        // Tolerances
        static final double ROTATION_TOLERANCE_DEG = 2.0;
        static final double STRAFE_TOLERANCE_DEG = 1.0;
        // Speed limits
        static final double MAX_ROTATION_SPEED = 1.0;
        static final double MAX_STRAFE_SPEED = 0.5;
    }

    // Next, add the constructor and PID configuration:
    public VisionCenterOnTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Initialize PID controllers
        rotationPID = new PIDController(
            Gains.ROTATION_P,
            Gains.ROTATION_I,
            Gains.ROTATION_D);
        
        strafePID = new PIDController(
            Gains.STRAFE_P,
            Gains.STRAFE_I,
            Gains.STRAFE_D);
            
        // Configure PID controllers
        rotationPID.setTolerance(Gains.ROTATION_TOLERANCE_DEG);
        strafePID.setTolerance(Gains.STRAFE_TOLERANCE_DEG);
        
        // Create robot-centric drive request
        drive = new SwerveRequest.RobotCentric();
        
        addRequirements(vision, drivetrain);
        
        // Add telemetry
        initializeDashboard();
    }

// Add initialization and dashboard setup:
    private void initializeDashboard() {
        SmartDashboard.putNumber("CenterTag/Rotation/kP", Gains.ROTATION_P);
        SmartDashboard.putNumber("CenterTag/Rotation/kD", Gains.ROTATION_D);
        SmartDashboard.putNumber("CenterTag/Strafe/kP", Gains.STRAFE_P);
        SmartDashboard.putNumber("CenterTag/Strafe/kD", Gains.STRAFE_D);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        strafePID.reset();
        
        // Update PID values from dashboard
        updatePIDValues();
    }

    private void updatePIDValues() {
        rotationPID.setPID(
            SmartDashboard.getNumber("CenterTag/Rotation/kP", Gains.ROTATION_P),
            0,
            SmartDashboard.getNumber("CenterTag/Rotation/kD", Gains.ROTATION_D));
            
        strafePID.setPID(
            SmartDashboard.getNumber("CenterTag/Strafe/kP", Gains.STRAFE_P),
            0,
            SmartDashboard.getNumber("CenterTag/Strafe/kD", Gains.STRAFE_D));
    }

    // Implement the execute method with combined rotation and strafe control:

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            stopMovement();
            return;
        }

        // Get current offsets from vision
        double horizontalOffset = vision.getHorizontalOffset();
        double tagId = vision.getTagId();
        
        // Validate tag ID
        if (tagId < 1 || tagId > 22) {
            stopMovement();
            return;
        }

        // Calculate rotation and strafe speeds with PID
        double rotationSpeed = rotationPID.calculate(horizontalOffset, 0);
        double strafeSpeed = strafePID.calculate(horizontalOffset, 0);
        
        // Apply speed limits
        rotationSpeed = Math.min(Math.abs(rotationSpeed), Gains.MAX_ROTATION_SPEED) 
                       * Math.signum(rotationSpeed);
        strafeSpeed = Math.min(Math.abs(strafeSpeed), Gains.MAX_STRAFE_SPEED) 
                     * Math.signum(strafeSpeed);

        // Update robot movement
        drivetrain.setControl(drive
            .withVelocityX(0.0)  // No forward/backward movement
            .withVelocityY(strafeSpeed)  // Strafe left/right
            .withANGULARRate(rotationSpeed));  // Rotate to align
            
        // Update telemetry
        updateTelemetry(horizontalOffset, rotationSpeed, strafeSpeed);
    }

    // Add helper methods for movement and telemetry:

    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withANGULARRate(0.0));
    }

    private void updateTelemetry(double offset, double rotationSpeed, double strafeSpeed) {
        SmartDashboard.putNumber("CenterTag/HorizontalOffset", offset);
        SmartDashboard.putNumber("CenterTag/RotationSpeed", rotationSpeed);
        SmartDashboard.putNumber("CenterTag/StrafeSpeed", strafeSpeed);
        SmartDashboard.putBoolean("CenterTag/ANGULARigned", rotationPID.atSetpoint());
        SmartDashboard.putBoolean("CenterTag/StrafeCentered", strafePID.atSetpoint());
    }
    // Finally, implement isFinished and end methods:

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || 
               (rotationPID.atSetpoint() && strafePID.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        SmartDashboard.putBoolean("CenterTag/Active", false);
    }
}
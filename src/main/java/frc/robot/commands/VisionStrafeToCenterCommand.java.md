package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionStrafeToCenterCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController strafePID;
    private final SwerveRequest.RobotCentric drive;

    // Tuning constants
    private static final double kP = 0.03; // Start conservative
    private static final double kI = 0.0;  // Likely not needed
    private static final double kD = 0.001; // Small amount for dampening
    
    // Tuning settings
    private static final double STRAFE_TOLERANCE = 1.0; // Degrees
    private static final double MAX_STRAFE_SPEED = 0.5; // Meters per second
    private static final double MIN_STRAFE_SPEED = 0.1; // Minimum speed to overcome friction

    public VisionStrafeToCenterCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Configure PID
        strafePID = new PIDController(kP, kI, kD);
        strafePID.setTolerance(STRAFE_TOLERANCE);
        
        // Create robot-centric drive request (robot's perspective, not field)
        drive = new SwerveRequest.RobotCentric();
        
        addRequirements(drivetrain);
        
        // Add dashboard values for tuning
        SmartDashboard.putNumber("Strafe/kP", kP);
        SmartDashboard.putNumber("Strafe/Tolerance", STRAFE_TOLERANCE);
    }

    @Override
    public void initialize() {
        strafePID.reset();
        // Update PID values from dashboard
        strafePID.setP(SmartDashboard.getNumber("Strafe/kP", kP));
        strafePID.setTolerance(SmartDashboard.getNumber("Strafe/Tolerance", STRAFE_TOLERANCE));
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withANGULARRate(0));
            return;
        }

        // Get horizontal offset from target
        double tx = vision.getHorizontalOffset();
        
        // Calculate strafe speed using PID
        double strafeSpeed = strafePID.calculate(tx, 0);
        
        // Apply speed constraints
        strafeSpeed = Math.copySign(
            Math.min(Math.abs(strafeSpeed), MAX_STRAFE_SPEED), 
            strafeSpeed);
            
        // Apply minimum speed if we're moving (helps overcome friction)
        if (Math.abs(strafeSpeed) > 0.01) {
            strafeSpeed = Math.copySign(
                Math.max(Math.abs(strafeSpeed), MIN_STRAFE_SPEED),
                strafeSpeed);
        }

        // Log data
        SmartDashboard.putNumber("Strafe/Offset", tx);
        SmartDashboard.putNumber("Strafe/Speed", strafeSpeed);
        SmartDashboard.putBoolean("Strafe/AtSetpoint", strafePID.atSetpoint());

        // Note: For a RobotCentric request
        // Positive Y = strafe left
        // Negative Y = strafe right
        // If target is to right (positive tx), we need to strafe left (positive Y)
        drivetrain.setControl(drive
            .withVelocityX(0)  // No forward/back movement
            .withVelocityY(strafeSpeed)  // Strafe speed (positive = left, negative = right)
            .withANGULARRate(0));  // No rotation
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || strafePID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withANGULARRate(0));
    }
}
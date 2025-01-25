package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// Based on Team Clockwork's code 2025
// https://claude.ai/chat/43c2b4d1-275a-4451-bfab-386ef595fccc

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.clockwork.ClockworkDriveConstants;
import frc.robot.subsystems.clockwork.ClockworkVisionSubsystem;

public class VisionCenterCommand_v6 extends Command {
    private final ClockworkVisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotationController;
    private final SwerveRequest.RobotCentric drive;
    
    // Rate limiters for smooth acceleration/deceleration
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

    public VisionCenterCommand_v6(ClockworkVisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        // Configure PID for rotation control
        rotationController = new PIDController(ClockworkDriveConstants.VISION_kP, 0, 0);
        rotationController.setTolerance(ClockworkDriveConstants.ROTATE_TOLERANCE);
        
        // Create robot-centric drive request for vision alignment
        drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            stopMovement();
            return;
        }

        // Calculate rotation speed based on target offset
        double rotationSpeed = calculateRotationSpeed();
        
        // Apply rate limiting for smooth motion
        rotationSpeed = rotationLimiter.calculate(rotationSpeed);

        // Update drive command with calculated speeds
        drivetrain.setControl(drive
            .withVelocityX(0)  // No forward/back movement during alignment
            .withVelocityY(0)  // No strafe during alignment
            .withRotationalRate(rotationSpeed));
        

        // Log alignment data
        logTelemetry(rotationSpeed);
    }

    private double calculateRotationSpeed() {
        double horizontalOffset = vision.getHorizontalOffset();
        
        // Calculate rotation speed with PID
        double rawSpeed = rotationController.calculate(horizontalOffset, 0);
        
        // Clamp speed to prevent excessive rotation
        return Math.min(Math.abs(rawSpeed), ClockworkDriveConstants.MAX_ANGULAR_RATE) 
               * Math.signum(rawSpeed);
    }

    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
    }

    private void logTelemetry(double rotationSpeed) {
        SmartDashboard.putNumber("Vision/RotationSpeed", rotationSpeed);
        SmartDashboard.putBoolean("Vision/AtSetpoint", rotationController.atSetpoint());
    }
}
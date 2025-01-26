package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// Based on Team Clockwork's code 2025
// https://claude.ai/chat/43c2b4d1-275a-4451-bfab-386ef595fccc

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionCenterCommand_v6 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotationController;
    private final SwerveRequest.RobotCentric drive;
        
    // Rate limiters for smooth acceleration/deceleration
    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

    public VisionCenterCommand_v6(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        rotationController = new PIDController(
            VisionConstants.ROTATIONAL_kP, 
            VisionConstants.ROTATIONAL_kI,
            VisionConstants.ROTATIONAL_kD
        );
        
        rotationController.setTolerance(VisionConstants.ROTATIONAL_TOLERANCE);
        
        drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(drivetrain);
        // initializeShuffleboard();
    }

    @Override
    public void initialize() {
        System.out.println("VisionCenterCommand initialized");
    }

    @Override 
    public void execute() {
        System.out.println("Vision has target: " + vision.hasTarget());
        System.out.println("Horizontal offset: " + vision.getHorizontalOffset());

         // Update PID values from dashboard
         double newP = SmartDashboard.getNumber("v6 Tran-kP", VisionConstants.TRANSLATIONAL_kP);
         double newI = SmartDashboard.getNumber("v6 Tran-kI", VisionConstants.TRANSLATIONAL_kP);
         double newD = SmartDashboard.getNumber("v6 Tran-kD", VisionConstants.TRANSLATIONAL_kD);
         
         if (newP != VisionConstants.TRANSLATIONAL_kP || newI != VisionConstants.TRANSLATIONAL_kD || newD != VisionConstants.TRANSLATIONAL_kD) {
            VisionConstants.TRANSLATIONAL_kP = newP;
            VisionConstants.TRANSLATIONAL_kI = newI; 
            VisionConstants.TRANSLATIONAL_kD = newD;
            rotationController.setPID(VisionConstants.TRANSLATIONAL_kP, VisionConstants.TRANSLATIONAL_kI, VisionConstants.TRANSLATIONAL_kD);
         }
        
        if (!vision.hasTarget()) {
            System.out.println("No target - stopping movement");
            stopMovement();
            return;
        }

        // Apply rate limiting for smooth motion
        double rotationSpeed = calculateRotationSpeed();

        System.out.println("Calculated rotation speed: " + rotationSpeed);
        
        rotationSpeed = rotationLimiter.calculate(rotationSpeed);
        System.out.println("Limited rotation speed: " + rotationSpeed);
        // Update drive command with calculated speeds
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0) 
            .withRotationalRate(rotationSpeed));

        // Log alignment data
        logTelemetry(rotationSpeed);
    }

    private double calculateRotationSpeed() {
        double horizontalOffset = vision.getHorizontalOffset();
        
        // Calculate rotation speed with PID
        double rawSpeed = rotationController.calculate(horizontalOffset, 0);
        
        // Clamp speed to prevent excessive rotation
        return Math.min(Math.abs(rawSpeed), VisionConstants.MAX_ANGULAR_VELOCITY) 
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
        SmartDashboard.putNumber("v6 RotationSpeed", rotationSpeed);
        SmartDashboard.putBoolean("v6 AtSetpoint", rotationController.atSetpoint());
    }
}
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.TOFSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class DecelerateRykerCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final TOFSubsystem tof;
    private final ProfiledPIDController distanceController;
    private final SwerveRequest.RobotCentric drive;

    // Tuning constants
    // Tune these constants to be more conservative
    private static final double MAX_VELOCITY = 1.0; // Reduced from 4.0 m/s
    private static final double MAX_ACCEL = 1.0; // Reduced from 3.0 m/s²
    private static final double TARGET_DISTANCE = 1.0; // meters
    private static final double DISTANCE_TOLERANCE = 0.1; // Increased from 0.05m
    
 // Start with just proportional control
 private static final double kP = 0.5; // Reduced from 1.0
 private static final double kI = 0.0;
 private static final double kD = 0.0;

   // Add debug logging
   private double lastCalculatedVelocity = 0.0;
   private double lastMeasuredDistance = 0.0;


    public DecelerateRykerCommand(CommandSwerveDrivetrain drivetrain, 
                                VisionSubsystem vision,
                                TOFSubsystem tof) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.tof = tof;
        
        // Configure profiled PID controller
        TrapezoidProfile.Constraints constraints = 
            new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCEL);
            
        distanceController = new ProfiledPIDController(kP, kI, kD, constraints);
        distanceController.setTolerance(DISTANCE_TOLERANCE);
        
        drive = new SwerveRequest.RobotCentric();
        
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
  // Reset controller with current distance
        lastMeasuredDistance = tof.getDistanceMeters();
        distanceController.reset(lastMeasuredDistance);
        
        System.out.println("DecelerateRyker initialized at distance: " + lastMeasuredDistance);    }

    @Override
    public void execute() {
  // Validate sensor data first
        if (!vision.hasTarget() || !tof.isRangeValid()) {
            stopMovement();
            // System.out.println("No valid target or TOF reading");
            return;
        }

        try {
            // Get current distance
            lastMeasuredDistance = tof.getDistanceMeters();
            
            // Calculate velocity with bounds checking
            lastCalculatedVelocity = distanceController.calculate(lastMeasuredDistance, TARGET_DISTANCE);
            lastCalculatedVelocity = MathUtil.clamp(lastCalculatedVelocity, -MAX_VELOCITY, MAX_VELOCITY);

            // Calculate rotation with bounds
            double rotationSpeed = -vision.getHorizontalOffset() * 0.05; // Reduced from 0.1
            rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);

            // Log before applying
            System.out.printf("Distance: %.2f, Velocity: %.2f, Rotation: %.2f%n", 
                            lastMeasuredDistance, lastCalculatedVelocity, rotationSpeed);

            // Apply control
            drivetrain.setControl(drive.withVelocityX(lastCalculatedVelocity)
                                     .withVelocityY(0)
                                     .withRotationalRate(rotationSpeed));

            // Update telemetry
            updateTelemetry();

        } catch (Exception e) {
            stopMovement();
            System.out.println("Error in DecelerateRyker execute: " + e.getMessage());
            e.printStackTrace();
        }    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || 
               !tof.isRangeValid() ||
               distanceController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0)
                                 .withVelocityY(0)
                                 .withRotationalRate(0));
    }

    private void stopMovement() {
        drivetrain.setControl(drive.withVelocityX(0)
                                 .withVelocityY(0)
                                 .withRotationalRate(0));
    }
    private void updateTelemetry() {
        SmartDashboard.putNumber("Decel/CurrentDistance", lastMeasuredDistance);
        SmartDashboard.putNumber("Decel/TargetDistance", TARGET_DISTANCE);
        SmartDashboard.putNumber("Decel/CalculatedVelocity", lastCalculatedVelocity);
        SmartDashboard.putBoolean("Decel/AtGoal", distanceController.atGoal());
        SmartDashboard.putBoolean("Decel/HasTarget", vision.hasTarget());
        SmartDashboard.putBoolean("Decel/ValidRange", tof.isRangeValid());
    }

}
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
    private static final double MAX_VELOCITY = 4.0; // meters/second  
    private static final double MAX_ACCEL = 3.0; // meters/second²
    private static final double TARGET_DISTANCE = 1.0; // meters
    private static final double DISTANCE_TOLERANCE = 0.05; // meters
    
    // PID Gains - will need tuning
    private static final double kP = 1.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

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
        distanceController.reset(tof.getDistanceMeters());
    }

    @Override
    public void execute() {
        if (!vision.hasTarget() || !tof.isRangeValid()) {
            drivetrain.setControl(drive.withVelocityX(0)
                                    .withVelocityY(0)
                                    .withRotationalRate(0));
            return;
        }

        // Calculate velocity based on current distance
        double currentDistance = tof.getDistanceMeters();
        double velocity = distanceController.calculate(currentDistance, TARGET_DISTANCE);

        // Keep robot aligned to target while moving
        double rotationSpeed = -vision.getHorizontalOffset() * 0.1; // Simple P control

        // Apply control
        drivetrain.setControl(drive.withVelocityX(velocity)
                                 .withVelocityY(0)
                                 .withRotationalRate(rotationSpeed));
                                 
        // Create or get the Shuffleboard tab
        ShuffleboardTab tab = Shuffleboard.getTab("DecelerateToTag");

        // Log data to Shuffleboard
        tab.add("Decel CurrentDistance", currentDistance);
        tab.add("Decel TargetVelocity", velocity);
        tab.add("Decel RotationSpeed", rotationSpeed);
    }

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
}
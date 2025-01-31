// https://claude.ai/chat/42c52596-bd4a-4b3b-bf8d-0876572f60df

Tips for Reliable AprilTag Tracking:


Filter bad measurements using confidence thresholds
Account for latency using timestamps
Test different resolutions/exposure settings
Consider using multiple pipelines for different ranges
Use field-relative control when possible
Add deadbands to prevent jitter
Test and tune PID values for smooth motion

This approach provides robust pose estimation by fusing odometry with vision while handling real-world challenges like latency and measurement uncertainty.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers; // Using Limelight's helper class
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final CommandSwerveDrivetrain drivetrain;

    public VisionSubsystem(String limelightName, CommandSwerveDrivetrain drivetrain) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Update dashboard with vision data
        SmartDashboard.putBoolean("Has Target", hasTarget());
        SmartDashboard.putNumber("Target ID", getTagId());
        SmartDashboard.putNumber("TX", getTX());
        SmartDashboard.putNumber("TY", getTY());
    }

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTX() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getTagId() {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public void setLeds(boolean enabled) {
        LimelightHelpers.setLEDMode(limelightName, enabled ? 3 : 1);
    }

    // Gets robot pose from Limelight
    public Pose2d getBotPose() {
        double[] botpose = LimelightHelpers.getBotpose(limelightName);
        if (botpose.length > 0) {
            return new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees(botpose[5]));
        }
        return null;
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

public class AlignToTargetCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;
    
    // PID Constants for alignment
    private static final double kP = 0.03;
    private static final double ALIGNMENT_TOLERANCE = 2.0; // degrees
    
    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Simple proportional control
            double tx = vision.getTX();
            double rotationSpeed = -tx * kP; // Negative because positive tx means target is to the right
            
            // Apply the rotation while keeping other axes still
            drivetrain.setControl(drive.withVelocityX(0)
                                     .withVelocityY(0)
                                     .withRotationalRate(rotationSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getTX()) < ALIGNMENT_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0)
                                 .withVelocityY(0)
                                 .withRotationalRate(0));
    }
}

public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem("limelight", drivetrain);

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Drive controls
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
                                             .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                             .withRotationalRate(-driver.getRightX() * MaxAngularRate))
        );

        // Vision alignment
        driver.a().whileTrue(new AlignToTargetCommand(vision, drivetrain));
        
        // Toggle Limelight LEDs
        operator.y().onTrue(runOnce(() -> vision.setLeds(true)))
                   .onFalse(runOnce(() -> vision.setLeds(false)));
    }
}


// Examples could include:
- Strafe to center on target (using tx)
- Drive to specific distance (using ty)
- Combined movements for scoring positions

// In your AutoRoutines class:
public Command driveAndAlign() {
    return Commands.sequence(
        // Drive to rough position using Choreo
        trajectoryCommand,
        // Fine align with vision
        new AlignToTargetCommand(vision, drivetrain)
    );
}
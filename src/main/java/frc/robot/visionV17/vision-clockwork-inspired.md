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



package frc.robot.commands;



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
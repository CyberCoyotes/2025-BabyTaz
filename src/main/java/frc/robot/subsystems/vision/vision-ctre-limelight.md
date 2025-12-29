# Yet Another attempt

Produced 2025-02-02 11pm

Reference
https://claude.ai/chat/d7d185dd-2777-4579-879f-85febce26cc1


```
public class AlignToTargetCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;
    
    // Use WPILib PID for vision alignment
    private final PIDController rotationPID;
    
    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1);

        // Configure WPILib PID
        rotationPID = new PIDController(0.035, 0.0001, 0.002);
        rotationPID.setTolerance(1.0); // Degrees
        rotationPID.enableContinuousInput(-180, 180); // Handle angle wrapping

        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            double tx = vision.getTargetX();
            double rotationSpeed = rotationPID.calculate(tx, 0);
            
            // Apply with rate limiting for smooth motion
            drivetrain.setControl(drive.withRotationalRate(rotationSpeed));
            
            // Log data for tuning
            SmartDashboard.putNumber("V22/Error", rotationPID.getPositionError());
            SmartDashboard.putNumber("V22/Output", rotationSpeed);
            SmartDashboard.putBoolean("V22/AtTarget", rotationPID.atSetpoint());
        }
    }
}
```
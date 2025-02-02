# Mechanical Advantage Approach

This approach is based on the Mechanical Advantage demo code for AprilTag tracking

// https://claude.ai/chat/e14926f1-c61b-4794-bee9-0068f59fc565

Show you a simplified version that would be more appropriate for a mid-level team while still using modern approaches? Keep in mind our hardware and software specifics: using CTRE TalonFX with Kraken motors and Tuner X generated swerve drivetrain. We are using a Limelight 3 for vision


public class RobotContainer {
    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem("limelight", drivetrain);
    private final LEDSubsystem leds = new LEDSubsystem();

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // When operator presses Y button, enable LEDs and vision
        operator.y().onTrue(Commands.runOnce(() -> vision.setLeds(true)))
                  .onFalse(Commands.runOnce(() -> vision.setLeds(false)));

        // When operator presses A button, center on target
        operator.a().whileTrue(new CenterOnTagCommand(drivetrain, vision));

        // When operator presses B button, align to target pose
        operator.b().whileTrue(new AlignToTargetCommand(drivetrain, vision));
    }
}

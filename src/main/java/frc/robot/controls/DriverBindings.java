package frc.robot.controls;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class DriverBindings {
    private final CommandXboxController driver;
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    public DriverBindings(CommandXboxController driver, CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.driver = driver;
        this.drivetrain = drivetrain;
        this.vision = vision;
        configureBindings();
    }

    private void configureBindings() {

        // Face Buttons
        // driver.a().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));
        // driverController.b().onTrue(null/* B Button Press */);
        // driverController.b().whileTrue(null/* B Button Hold */);
        // driverController.x().onTrue(null/* X Button Press */);
        // driverController.x().whileTrue(null/* X Button Hold */);
        // driverController.y().onTrue(null/* Y Button Press */);
        // driverController.y().whileTrue(null/* Y Button Hold */);

        // Bumpers
        // driverController.leftBumper().onTrue(/* Left Bumper Press */);
        // driverController.leftBumper().whileTrue(/* Left Bumper Hold */);
        // driverController.rightBumper().onTrue(/* Right Bumper Press */);
        // driver.rightBumper().whileTrue(new VisionAlignToTargetCommand(vision, drivetrain));

        // Triggers
        // driverController.leftTrigger().onTrue(/* Left Trigger Press */);
        // driverController.leftTrigger().whileTrue(/* Left Trigger Hold */);
        // driverController.rightTrigger().onTrue(/* Right Trigger Press */);
        // driverController.rightTrigger().whileTrue(/* Right Trigger Hold */);

        // D-Pad
        // driverController.povUp().onTrue(/* D-Pad Up Press */);
        // driverController.povRight().onTrue(/* D-Pad Right Press */);
        // driverController.povDown().onTrue(/* D-Pad Down Press */);
        // driverController.povLeft().onTrue(/* D-Pad Left Press */);

        // Start/Back
        // driverController.start().onTrue(/* Start Button Press */);
        // driverController.back().onTrue(/* Back Button Press */);
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision18.VisionSubsystem;


public class RobotContainer {

    private double slowMo = 0.30;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)* slowMo; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * slowMo; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    
    public RobotContainer() {
        configureBindings();
    
     }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        /* driver.a().whileTrue(drivetrain.applyRequest(() -> brake)); */
        /* driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        )); */


        // reset the field-centric heading on left bumper press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // AprilTag alignment controls
        // A Button: Start alignment to AprilTag (full 3-axis: forward/back, left/right, rotation)
        // B Button: Stop/cancel alignment (interrupts the command)
        driver.a().whileTrue(new AlignToAprilTagCommand(drivetrain, vision18));
        driver.b().onTrue(Commands.runOnce(() -> {
            // Cancel any running alignment command
            if (drivetrain.getCurrentCommand() != null) {
                drivetrain.getCurrentCommand().cancel();
            }
        }, drivetrain));

        // Legacy alignment commands (commented out - keeping for reference)
        // driver.a().whileTrue(new AlignToTagCommand18f(drivetrain, vision18));
        // driver.b().whileTrue(new AlignToTagCommand18b(drivetrain, vision18));
        // driver.x().whileTrue(new AlignToTagCommand18x(drivetrain, vision18));
        // driver.y().whileTrue(new AlignToTagCommand18y(drivetrain, vision18));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

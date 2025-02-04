// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision18.AlignToTagCommand18a;
import frc.robot.subsystems.vision18.AlignToTagCommand18b;
import frc.robot.subsystems.vision18.AlignToTagCommand18x;
import frc.robot.subsystems.vision18.AlignToTagCommand18y;
import frc.robot.subsystems.vision18.ForwardDistanceTest;
import frc.robot.subsystems.vision18.VisionSubsystem18;


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

    // TODO Create a VisionSubsystem18 object
    private final VisionSubsystem18 vision18 = new VisionSubsystem18("limelight", drivetrain);

    
    public RobotContainer() {
        configureBindings();
            // Setup vision logging
        VisionSubsystem18.setupAdvantagescopeLayout();
       // Configure telemetry updates
    //    vision18.setLoggingEnabled(true);
    
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // TODO Add a command to align to the AprilTag when the A button is pressed
        driver.a().whileTrue(new ForwardDistanceTest(drivetrain, vision18));
        driver.b().whileTrue(new AlignToTagCommand18b(drivetrain, vision18));
        driver.x().whileTrue(new AlignToTagCommand18x(drivetrain, vision18));
        driver.y().whileTrue(new AlignToTagCommand18y(drivetrain, vision18));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

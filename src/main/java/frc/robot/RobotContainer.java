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
import frc.robot.commands.visiontest.FullAlignToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTestDashboard;


public class RobotContainer {

    private double slowMo = 0.50;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Vision subsystem for AprilTag detection with state tracking and LED feedback
    // Phase 1: Merged from LimelightVision.java for unified vision subsystem
    private final VisionSubsystem vision = new VisionSubsystem("limelight");

    // Vision Test Dashboard - provides Shuffleboard buttons for testing vision models
    // Access via Shuffleboard tab "Vision Tests"
    private final VisionTestDashboard visionTestDashboard;


    public RobotContainer() {
        // Initialize vision test dashboard (creates Shuffleboard tab with buttons)
        visionTestDashboard = new VisionTestDashboard(drivetrain, vision);

        configureBindings();

     }


    private void configureBindings() {
        // Set vision subsystem to run continuously (just runs periodic for logging)
        vision.setDefaultCommand(Commands.run(() -> {}, vision));

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

        // ============================================================================
        // VISION TEST BINDINGS
        // ============================================================================
        // Primary: Use Shuffleboard "Vision Tests" tab for button controls
        // Backup: Controller buttons below (optional, can be commented out)
        //
        // Shuffleboard buttons are recommended for cleaner testing workflow.
        // See VisionTestDashboard.java for Shuffleboard layout.
        // ============================================================================

        // A button: Original AlignToTag (3-axis alignment) - KEEP FOR CONTINUED TESTING
        driver.x().whileTrue(new FullAlignToTag(drivetrain, vision));

        // B button: Model A - Rotation only alignment
        driver.a().whileTrue(visionTestDashboard.getModelACommand());

        // X button: Model B - Rotation + Range alignment
        driver.b().whileTrue(visionTestDashboard.getModelBCommand());

        // Y button: Model C - Perpendicular + Range alignment
        driver.y().whileTrue(visionTestDashboard.getModelCCommand());

        // Left Bumper: Model D - Color blob hunt and seek
        driver.leftBumper().whileTrue(visionTestDashboard.getModelDCommand());

        // Right Bumper: Stop all vision tests
        driver.rightBumper().onTrue(visionTestDashboard.getStopCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser; // TODO added Choreo import
import choreo.auto.AutoFactory; // TODO added Choreo import
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import frc.robot.commands.AlignToTargetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    // Drive constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second                                                                                    // max angular velocity
    private final double DEADBAND = 0.1; // 10% deadband

    // Controller setup
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final LEDSubsystem leds = new LEDSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem("limelight", drivetrain, leds);

    // Drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DEADBAND)
            .withRotationalDeadband(MaxAngularRate * DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        
    /* Auto Related */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        configureAutoRoutines();
        configureBindings();
        configureTelemetry();

    }
    
    private void configureAutoRoutines() {
        autoChooser.addRoutine("Two Meters", autoRoutines::twoMeters);
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        configureDrivetrainDefault();
        configureDriverBindings(); // Driver specific bindings
        configureOperatorBindings(); // Operator specific bindings
    }

    private void configureDrivetrainDefault() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
    
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDriverBindings() {
        // Basic driving
        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() -> 
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        // Vision alignment
        driver.rightBumper().whileTrue(new AlignToTargetCommand(vision, drivetrain));

        // Field-centric reset
        driver.leftBumper().onTrue(runOnce(() -> drivetrain.seedFieldCentric()));

        // SysId testing 
        configureSysIdBindings();

    }

    private void configureOperatorBindings() {
        // Add operator controls here
        operator.y().onTrue(runOnce(() -> vision.setLeds(true)))
                  .onFalse(runOnce(() -> vision.setLeds(false)));

    }
    
    private void configureSysIdBindings() {
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    private void configureTelemetry() {
    // Existing telemetry
    drivetrain.registerTelemetry(logger::telemeterize);
    SmartDashboard.putData("Vision Subsystem", vision);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    // Add vision telemetry
    SmartDashboard.putBoolean("Has Target", vision.hasTarget());
    SmartDashboard.putNumber("Target ID", vision.getTagId());
    // SmartDashboard.putNumber("Vision Latency", vision.getLatency());
    }


    public Command getAutonomousCommand() {
        // Return the command to run in autonomous
        return autoChooser.selectedCommand();
    }

}
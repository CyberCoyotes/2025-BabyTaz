// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.AutoAlignSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Controller Ports
    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int OPERATOR_CONTROLLER_PORT = 1;

    // Speed Constants
    private final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private final double MaxAngularRate = 1.5 * Math.PI;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    
    // IO
    private final LEDIO leds = new LED(27); // CANdle ID
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain, leds);


    // Drive Requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Configure default commands
        configureDefaultCommands();
        
        // Configure driver buttons
        configureDriverBindings();
        
        // Configure operator buttons
        configureOperatorBindings();
    }

    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );
    }

    private void configureDriverBindings() {
        // Face Buttons
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
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
        driverController.rightBumper().whileTrue(new AlignToTargetCommand(vision, drivetrain));

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

    private void configureOperatorBindings() {
        // Face Buttons
        // operatorController.a().onTrue(/* A Button Press */);
        // operatorController.a().whileTrue(/* A Button Hold */);
        // operatorController.b().onTrue(/* B Button Press */);
        // operatorController.b().whileTrue(/* B Button Hold */);
        // operatorController.x().onTrue(/* X Button Press */);
        // operatorController.x().whileTrue(/* X Button Hold */);
        // operatorController.y().onTrue(/* Y Button Press */);
        // operatorController.y().whileTrue(/* Y Button Hold */);

        // Bumpers
        // operatorController.leftBumper().onTrue(/* Left Bumper Press */);
        // operatorController.leftBumper().whileTrue(/* Left Bumper Hold */);
        // operatorController.rightBumper().onTrue(/* Right Bumper Press */);
        // operatorController.rightBumper().whileTrue(/* Right Bumper Hold */);

        // Triggers
        // operatorController.leftTrigger().onTrue(/* Left Trigger Press */);
        // operatorController.leftTrigger().whileTrue(/* Left Trigger Hold */);
        // operatorController.rightTrigger().onTrue(/* Right Trigger Press */);
        // operatorController.rightTrigger().whileTrue(/* Right Trigger Hold */);

        // D-Pad
        // operatorController.povUp().onTrue(/* D-Pad Up Press */);
        // operatorController.povRight().onTrue(/* D-Pad Right Press */);
        // operatorController.povDown().onTrue(/* D-Pad Down Press */);
        // operatorController.povLeft().onTrue(/* D-Pad Left Press */);
        
        // Start/Back
        // operatorController.start().onTrue(/* Start Button Press */);
        // operatorController.back().onTrue(/* Back Button Press */);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
    
        // Example autonomous that aligns to a scoring position
            Pose2d scoringPose = new Pose2d(14.0, 5.5, Rotation2d.fromDegrees(180));
            
            return new AutoAlignSequence(vision, drivetrain, scoringPose);
    }
    
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.experimental.VisionTestLevel1Command;
import frc.robot.experimental.VisionTestLevel2Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystickOne = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain); // My vision subsystem
  private final NoteDetectionSubsystem m_noteDetection = new NoteDetectionSubsystem(drivetrain); // Note detection sub                                                                                                // subsystem
  private final LEDSubsystem leds = new LEDSubsystem(30); // Replace 0 with your CANdle ID


  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystickOne.getLeftY() * MaxSpeed) // Drive forward with
            // negative Y (forward)
            .withVelocityY(-joystickOne.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystickOne.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X
                                                                           // (left)
        ));

    joystickOne.start().whileTrue(drivetrain.applyRequest(() -> brake));
    joystickOne.y().whileTrue(drivetrain
        .applyRequest(
            () -> point.withModuleDirection(new Rotation2d(-joystickOne.getLeftY(), -joystickOne.getLeftX()))));

           
// Test vision Level 1 with A button
joystickOne.a().whileTrue(new VisionTestLevel1Command(m_vision, leds, drivetrain));

// Test vision Level 2 with B button
joystickOne.b().whileTrue(new VisionTestLevel2Command(m_vision, leds, drivetrain));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoAlignSequence;
import frc.robot.controls.ControlsTelemetry;
import frc.robot.controls.DriverBindings;
import frc.robot.controls.OperatorBindings;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.VirtualControlPanel;

public class RobotContainer {

    // TODO Virtual Controls
    private final VirtualControlPanel virtualControls;

    // New fields were recommended to be added here to prevent garbage collection
    private final DriverBindings driverBindings;
    private final OperatorBindings operatorBindings;

    // Controller Ports
    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int OPERATOR_CONTROLLER_PORT = 1;

    private final ControlsTelemetry telemetry;


    // Speed Constants
    private final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private final double MaxAngularRate = 1.5 * Math.PI;

    // Controllers
    private final CommandXboxController driver = new CommandXboxController(DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operator = new CommandXboxController(OPERATOR_CONTROLLER_PORT);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final LEDSubsystem leds = new LEDSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem("limelight", drivetrain, leds);

    // Drive Requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        telemetry = new ControlsTelemetry(driver, operator);
        virtualControls = new VirtualControlPanel();


        // Actual configuration of the button bindings for each controller is handled in the DriverBindings and OperatorBindings classes
        driverBindings = new DriverBindings(driver, drivetrain, vision);
        operatorBindings = new OperatorBindings(operator);

        // Configure the default commands
        configureDefaultCommands();

    }


    private void configureDefaultCommands() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driver.getLeftY() * MaxSpeed)
                .withVelocityY(-driver.getLeftX() * MaxSpeed)
                .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );
    }

    

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
    
        // Example autonomous that aligns to a scoring position
            Pose2d scoringPose = new Pose2d(14.0, 5.5, Rotation2d.fromDegrees(180));
            
            return new AutoAlignSequence(vision, drivetrain, scoringPose);
    }
    
}
/*
The key components of the current structure are:

1.Field Organization
* Controllers and bindings together at top
* Subsystems grouped together
* Drive requests grouped together

2.Dependency Flow
* Subsystems created first
* Bindings receive required dependencies
* Default commands configured last

3.Main Functions
* Constructor handles initialization order
* Default commands handle base driving
* Autonomous command separate for competition

 */
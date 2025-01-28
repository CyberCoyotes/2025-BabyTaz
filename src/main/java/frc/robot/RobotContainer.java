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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.auto.AutoRoutines;
import frc.robot.commands.VisionCenterCommand_v5;
import frc.robot.commands.AlignFrontBackCommand;
import frc.robot.commands.AlignStrafeCommand;
import frc.robot.commands.VisionCenterCommand_v10;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TOFSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
    private final Pose2d targetPose;

    // Drive constants
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1.0).in(RadiansPerSecond); // 3/4 of a rotation per second                                                                                    // max angular velocity
    private final double DEADBAND = 0.1; // 10% deadband

    // Add these speed factor variables
    private double driveSpeedFactor = 0.25; // 25% speed for rookie drivers
    private double rotationSpeedFactor = 0.5; // 25% rotation speed

        // Controller setup
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    // Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final LEDSubsystem leds = new LEDSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem("limelight", leds);
    private final TOFSubsystem tof = new TOFSubsystem(); // TODO Run configuration for TOF sensor to confirm

  // TODO Emergency stop trigger based on TOF distance
    // private final Trigger emergencyStop = new Trigger(() -> 
        // tof.isRangeValid() && tof.getDistanceMeters() < 100.0); // 30cm minimum


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


    // TODO Add turret subsystem
    private final TurretSubsystem turret = new TurretSubsystem(27); 

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public RobotContainer() {
        targetPose = new Pose2d(); // Initialize targetPose
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory, drivetrain, turret);


        configureAutoRoutines();
        configureBindings();
        configureTelemetry();

    }
    private void configureAutoRoutines() {
        
        // DEFAULT without scoring 
        autoChooser.addRoutine("Drive Forward", autoRoutines::driveForward);

        /****   START CENTER    ****/
        // autoChooser.addRoutine("Center H1", autoRoutines::centerH); // FIXME Add this routine
        // autoChooser.addRoutine("Center H2", autoRoutines::centerH); // FIXME Add this routine
        // autoChooser.addRoutine("Center H4", autoRoutines::centerH); // FIXME Add this routine <--
        // autoChooser.addRoutine("Center G1", autoRoutines::centerG); // FIXME Add this routine
        // autoChooser.addRoutine("Center G2", autoRoutines::centerG); // FIXME Add this routine
        // autoChooser.addRoutine("Center G4", autoRoutines::centerG); // FIXME Add this routine <--


        /****   START TOP-BARGE   ****/
        autoChooser.addRoutine("Top L1", autoRoutines::topL);    // FIXME Finish this routine
        autoChooser.addRoutine("Top L2", autoRoutines::topL);    // FIXME Finish this routine
        autoChooser.addRoutine("Top L4", autoRoutines::topL);    // FIXME Finish this routine <--
        autoChooser.addRoutine("Top K1", autoRoutines::topK);    // FIXME Finish this routine
        autoChooser.addRoutine("Top K2", autoRoutines::topK);    // FIXME Finish this routine
        autoChooser.addRoutine("Top K4", autoRoutines::topK);    // FIXME Finish this routine <--


        autoChooser.addRoutine("Top J1", autoRoutines::topJ);    // FIXME Finish this routine
        autoChooser.addRoutine("Top J1-K4-L4-A4", autoRoutines::topJ); // FIXME Finish this routine
        autoChooser.addRoutine("Top J4-K4-L4-A4", autoRoutines::topJ); // FIXME Finish this routine <--
        // autoChooser.addRoutine("Top I1", autoRoutines::topI); // FIXME Add this routine
        
        // autoChooser.addRoutine("Top H", autoRoutines::topH); // CENTER Barge Side
        // autoChooser.addRoutine("Top G", autoRoutines::topG); // CENTER Barge Side
        // autoChooser.addRoutine("Top B", autoRoutines::topB); // CENTER Driver Side
        // autoChooser.addRoutine("Top A", autoRoutines::topA); // CENTER Driver Side
        
        /****   START BOTTOM-NONBARGE    ****/
        autoChooser.addRoutine("Bottom C1", autoRoutines::bottomC);  // FIXME Finish this routine
        autoChooser.addRoutine("Bottom C2", autoRoutines::bottomC);  // FIXME Finish this routine
        autoChooser.addRoutine("Bottom C4", autoRoutines::bottomC);  // FIXME Finish this routine <--

        autoChooser.addRoutine("Bottom D1", autoRoutines::bottomD);  // FIXME Finish this routine
        autoChooser.addRoutine("Bottom D2", autoRoutines::bottomD);  // FIXME Finish this routin
        autoChooser.addRoutine("Bottom D4", autoRoutines::bottomD);  // FIXME Finish this routine <--
        
        autoChooser.addRoutine("Bottom E1", autoRoutines::bottomE);  // FIXME Finish this routine
        autoChooser.addRoutine("Bottom E1-D4-C4-B4", autoRoutines::bottomE);  // FIXME Finish this routine
        autoChooser.addRoutine("Bottom E4-D4-C4-B4", autoRoutines::bottomE);  // FIXME Finish this routine <--
        // autoChooser.addRoutine("Bottom F1", autoRoutines::bottomF); // FIXME Add this routine
        

        // autoChooser.addRoutine("Bottom B", autoRoutines::bottomB); // CENTER Driver Side
        // autoChooser.addRoutine("Bottom A", autoRoutines::bottomA); // CENTER Driver Side
        // autoChooser.addRoutine("Bottom H", autoRoutines::bottomH); // CENTER Barge Side
        // autoChooser.addRoutine("Bottom G", autoRoutines::bottomG); // CENTER Barge Side


        
        SmartDashboard.putData("Autonomous", autoChooser);
        
        // Shuffleboard Setup
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
        autoTab.add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kCommand);    
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
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * driveSpeedFactor) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * driveSpeedFactor) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * rotationSpeedFactor) // Drive counterclockwise with negative X (left)
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

        // TODO Both are working. Both are aggressive, and overshoot but not wildly overshooting to the point of "shooting off" down the hall, lol
        // I think it really is an issue of tuning now!
        // Vision alignment
        driver.rightBumper().whileTrue(
            new VisionCenterCommand_v10(vision, drivetrain, driver));
        
        driver.leftBumper().whileTrue(
            new VisionCenterCommand_v5(vision, drivetrain));

        // Strafe alignment
        driver.povLeft().onTrue(new AlignStrafeCommand(drivetrain, AlignStrafeCommand.StrafeDirection.LEFT));
        driver.povRight().onTrue(new AlignStrafeCommand(drivetrain, AlignStrafeCommand.StrafeDirection.RIGHT));
        
        driver.povUp().onTrue(new AlignFrontBackCommand(drivetrain, AlignFrontBackCommand.StrafeDirection.FORWARD));
        driver.povDown().onTrue(new AlignFrontBackCommand(drivetrain, AlignFrontBackCommand.StrafeDirection.BACKWARD));
        
        // SysId testing 
        configureSysIdBindings();

    }

    private void configureOperatorBindings() {
        // Add operator controls here
        // operator.y().onTrue(runOnce(() -> vision.setLimelightLeds(true)))
                //   .onFalse(runOnce(() -> vision.setLimelightLeds(false)));

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

    // Add speed factor telemetry
    SmartDashboard.putNumber("Drive Speed Factor %", driveSpeedFactor * 100);
    SmartDashboard.putNumber("Rotation Speed Factor %", rotationSpeedFactor * 100);
    }


    public Command getAutonomousCommand() {
        // Return the command to run in autonomous
        return autoChooser.selectedCommand();
    }

}
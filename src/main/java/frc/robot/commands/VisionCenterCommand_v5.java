package frc.robot.commands;

/*
 * Reference
 * https://claude.ai/chat/1ca7b94d-e659-4ac5-8f29-01629b1543ce 
 */

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionCenterCommand_v5 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;

    // Updated for CTRE Swerve
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo); // TODO Hoping this is the secret sauce!
    
        private static final class Constraints {
            // Adjust these based on your robot's capabilities
        static final double MAX_VELOCITY = 1.0; // Start conservative
        static final double MAX_ACCELERATION = 2.0;
        static final double MAX_ROTATION_VELOCITY = Math.PI/2; // ~90 deg/sec
        static final double MAX_ROTATION_ACCELERATION = Math.PI;
    
            
            static final double POSITION_TOLERANCE = 0.02; // meters
            static final double ROTATION_TOLERANCE = 0.02; // radians
            
            static final double TRANSLATION_P = 0.5;
            static final double TRANSLATION_I = 0.0;
            static final double TRANSLATION_D = 0.0;
            static final double ROTATION_P = 1.0;
            static final double ROTATION_I = 0.0;
            static final double ROTATION_D = 0.05;
        }
    
        // Shuffleboard tuning entries
        private final GenericEntry pGainEntry = Shuffleboard.getTab("Vision")
            .add("Translation P", Constraints.TRANSLATION_P).getEntry();
        private final GenericEntry rotPGainEntry = Shuffleboard.getTab("Vision")
            .add("Rotation P", Constraints.ROTATION_P).getEntry();
    
        private final ProfiledPIDController strafeController;
        private final ProfiledPIDController rotationController;
    
        // public VisionImprovedCenterCommand_v5(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
    
        public VisionCenterCommand_v5(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
            this.vision = vision;
            this.drivetrain = drivetrain;
            // this.drive = new SwerveRequest.RobotCentric();

        TrapezoidProfile.Constraints translationConstraints = 
            new TrapezoidProfile.Constraints(Constraints.MAX_VELOCITY, Constraints.MAX_ACCELERATION);
        TrapezoidProfile.Constraints rotationConstraints = 
            new TrapezoidProfile.Constraints(Constraints.MAX_ROTATION_VELOCITY, Constraints.MAX_ROTATION_ACCELERATION);

        strafeController = new ProfiledPIDController(
            Constraints.TRANSLATION_P,
            Constraints.TRANSLATION_I, 
            Constraints.TRANSLATION_D,
            translationConstraints,
            0.02
        );

        rotationController = new ProfiledPIDController(
            Constraints.ROTATION_P,
            Constraints.ROTATION_I,
            Constraints.ROTATION_D,
            rotationConstraints,
            0.02
        );

                // Create Shuffleboard tab for tuning
        ShuffleboardTab tab = Shuffleboard.getTab("Vision Tuning");
        // Constraints
        tab.addNumber("Max Velocity", () -> Constraints.MAX_VELOCITY)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 2.0));
        
        tab.addNumber("Max Accel", () -> Constraints.MAX_ACCELERATION)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 3.0));

        tab.addNumber("Max Rotation Velocity", () -> Constraints.MAX_ROTATION_VELOCITY)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0.0, "max", 3.0));

        tab.addNumber("Max Rotation Accel", () -> Constraints.MAX_ROTATION_ACCELERATION)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0.0, "max", 3.0));
        // Tolerances
        tab.addNumber("Position Tolerance", () -> Constraints.POSITION_TOLERANCE)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.01, "max", 0.2));
        tab.addNumber("Rotation Tolerance", () -> Constraints.ROTATION_TOLERANCE)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.01, "max", 0.2));
        
        // Gains
        tab.addNumber("Translation P", () -> Constraints.TRANSLATION_P)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 0.5));
        tab.addNumber("Translation I", () -> Constraints.TRANSLATION_I)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0.0, "max", 0.5));
        tab.addNumber("Translation D", () -> Constraints.TRANSLATION_D)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.5));
        tab.addNumber("Rotation P", () -> Constraints.ROTATION_P)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 2.0));
        tab.addNumber("Rotation I", () -> Constraints.ROTATION_I)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", 0.0, "max", 2.0));
        tab.addNumber("Rotation D", () -> Constraints.ROTATION_D)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0.0, "max", 2.0));

            
        configurePIDControllers();
        addRequirements(vision, drivetrain);
    }

    private void configurePIDControllers() {
        strafeController.setTolerance(Constraints.POSITION_TOLERANCE);
        rotationController.setTolerance(Constraints.ROTATION_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        strafeController.reset(0, 0);
        rotationController.reset(0, 0);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            stopMovement();
            return;
        }

        int tagId = (int) vision.getTagId();
        if (tagId < 1 || tagId > 22) {
            stopMovement();
            return;
        }

        double horizontalOffset = vision.getHorizontalOffset();
        
        // Scale gains based on distance
        double distanceScale = Math.abs(horizontalOffset) / 10.0;
        distanceScale = Math.max(0.3, distanceScale); 

        // Update PID gains from dashboard and scale them
        strafeController.setP(pGainEntry.getDouble(Constraints.TRANSLATION_P) * distanceScale);
        rotationController.setP(rotPGainEntry.getDouble(Constraints.ROTATION_P) * distanceScale);

        double strafeSpeed = strafeController.calculate(horizontalOffset, 0);
        double rotationRate = rotationController.calculate(Units.degreesToRadians(horizontalOffset), 0);

        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(rotationRate));

        updateTelemetry(horizontalOffset, strafeSpeed, rotationRate);
    }

    private void updateTelemetry(double offset, double strafeSpeed, double rotationRate) {
        SmartDashboard.putNumber("v5/HorizontalOffset", offset);
        SmartDashboard.putNumber("v5/StrafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("v5/RotationRate", rotationRate);
        SmartDashboard.putNumber("v5/StrateError", strafeController.getPositionError());
        SmartDashboard.putNumber("v5/RotationError", rotationController.getPositionError());
        SmartDashboard.putBoolean("v5/AtTarget", isFinished());
        SmartDashboard.putString("v5/DriveRequestType", drive.DriveRequestType.toString());
        SmartDashboard.putString("v5/SteerRequestType", drive.SteerRequestType.toString());
        SignalLogger.writeDouble("v5/HorizontalOffset", offset);
        SignalLogger.writeDouble("v5/StrafeSpeed", strafeSpeed);
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || 
               vision.getState() == VisionState.TARGET_LOCKED ||
               (strafeController.atGoal() && rotationController.atGoal());
    }

    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0) 
            .withRotationalRate(0));
    }
    
    @Override
    public void end(boolean interrupted) {
        stopMovement();
        // strafeController.reset();
        // rotationController.reset();
        strafeController.reset(new TrapezoidProfile.State(0, 0));
        rotationController.reset(new TrapezoidProfile.State(0, 0));

        // Optional: Call drivetrain stop command
        drivetrain.stop().schedule();
    }
    
    
}
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Map;

public class VisionCenterCommand_v4 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;

    // Tunable values
    private double maxVelocity = 0.25;
    private double maxAcceleration = 0.5;
    private double maxRotationVelocity = 2.0;
    private double maxRotationAcceleration = 3.0;
    private double positionTolerance = 0.10;
    private double rotationTolerance = 0.02;
    private double translationP = 0.10;
    private double translationI = 0.0;
    private double translationD = 0.0;
    private double rotationP = 1.0;
    private double rotationI = 0.0;
    private double rotationD = 0.1;

    // Profiled PID Controllers
    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController rotationController;

    public VisionCenterCommand_v4(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.RobotCentric();

        // Initialize controllers
        strafeController = new ProfiledPIDController(
            translationP,
            translationI,
            translationD,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration),
            0.02
        );

        rotationController = new ProfiledPIDController(
            rotationP,
            rotationI,
            rotationD,
            new TrapezoidProfile.Constraints(maxRotationVelocity, maxRotationAcceleration),
            0.02
        );

        // Create Shuffleboard tab for tuning
        ShuffleboardTab tab = Shuffleboard.getTab("Vision Tuning");
        
        tab.addNumber("Max Velocity", () -> maxVelocity)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 1.0));
        
        tab.addNumber("Max Rotation Velocity", () -> maxRotationVelocity)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 3.0));
        
        tab.addNumber("Position Tolerance", () -> positionTolerance)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.01, "max", 0.2));
        
        tab.addNumber("Translation P", () -> translationP)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 0.5));
        
        tab.addNumber("Rotation P", () -> rotationP)
           .withWidget(BuiltInWidgets.kNumberSlider)
           .withProperties(Map.of("min", 0.0, "max", 2.0));

        configurePIDControllers();
        addRequirements(vision, drivetrain);
    }

    private void configurePIDControllers() {
        strafeController.setTolerance(positionTolerance);
        rotationController.setTolerance(rotationTolerance);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        strafeController.setP(translationP);
        strafeController.setI(translationI);
        strafeController.setD(translationD);

        rotationController.setP(rotationP);
        rotationController.setI(rotationI);
        rotationController.setD(rotationD);
    }

    @Override
    public void initialize() {
        strafeController.reset(new TrapezoidProfile.State());
        rotationController.reset(new TrapezoidProfile.State());
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

        // Get vision measurements
        double horizontalOffset = vision.getHorizontalOffset();

        // Scale gains down as we get closer to target
        double distanceScale = Math.abs(horizontalOffset) / 10.0;
        distanceScale = Math.max(0.3, distanceScale);

        // Update PID gains
        configurePIDControllers();

        // Calculate control outputs with scaling
        double strafeSpeed = strafeController.calculate(horizontalOffset, 0) * distanceScale;
        double rotationRate = rotationController.calculate(horizontalOffset, 0) * distanceScale;

        // Apply movement
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(rotationRate));

        updateTelemetry(horizontalOffset, strafeSpeed, rotationRate);
    }

    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private void updateTelemetry(double offset, double strafeSpeed, double rotationRate) {
        SmartDashboard.putNumber("Vision/HorizontalOffset", offset);
        SmartDashboard.putNumber("Vision/StrafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("Vision/RotationRate", rotationRate);
        SmartDashboard.putBoolean("Vision/AtTarget", isFinished());
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || 
               vision.getState() == VisionState.TARGET_LOCKED ||
               (strafeController.atGoal() && rotationController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        strafeController.reset(new TrapezoidProfile.State());
        rotationController.reset(new TrapezoidProfile.State());
    }
}
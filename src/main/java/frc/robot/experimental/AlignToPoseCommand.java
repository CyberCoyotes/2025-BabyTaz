package frc.robot.experimental;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Command to align robot to a specified pose using AprilTag detection.
 * Maintains a fixed distance while centering on target.
 */
public class AlignToPoseCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Pose2d targetPose;
    private final ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    // Add these near your other constants/Default class
    private static final boolean LIMELIGHT_MOUNTED_ON_BACK = true;
    private static final double DIRECTION_MULTIPLIER = LIMELIGHT_MOUNTED_ON_BACK ? -1.0 : 1.0;

    // PID Controllers for each axis of movement
    private final PIDController xController; // Forward/backward
    private final PIDController yController; // Left/right
    private final PIDController rotationController; // Rotation alignment

    // Swerve drive request
    private final SwerveRequest.RobotCentric drive;

    // Default PID values - will be overridden by SmartDashboard values
    private static final class Defaults {
        static final double TRANSLATION_P = 1.0;
        static final double TRANSLATION_I = 0.0;
        static final double TRANSLATION_D = 0.0;
        static final double ROTATION_P = 0.05;
        static final double ROTATION_I = 0.0;
        static final double ROTATION_D = 0.0;
        static final double TRANSLATION_TOLERANCE = 0.02; // meters
        static final double ROTATION_TOLERANCE = 2.0; // degrees
        static final double TARGET_DISTANCE = 0.2; // TODO: Set to actual distance
        static final double MAX_TRANSLATION_SPEED = 2.0; // meters per second
        static final double MAX_ROTATION_SPEED = 2.0; // radians per second

    }

    public AlignToPoseCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        // Initialize SmartDashboard values with defaults
        initializeSmartDashboard();
        configureShuffleboard();

        // Initialize PID controllers
        xController = new PIDController(
                SmartDashboard.getNumber("Align/Translation/kP", Defaults.TRANSLATION_P),
                SmartDashboard.getNumber("Align/Translation/kI", Defaults.TRANSLATION_I),
                SmartDashboard.getNumber("Align/Translation/kD", Defaults.TRANSLATION_D));

        yController = new PIDController(
                SmartDashboard.getNumber("Align/Translation/kP", Defaults.TRANSLATION_P),
                SmartDashboard.getNumber("Align/Translation/kI", Defaults.TRANSLATION_I),
                SmartDashboard.getNumber("Align/Translation/kD", Defaults.TRANSLATION_D));

        rotationController = new PIDController(
                SmartDashboard.getNumber("Align/Rotation/kP", Defaults.ROTATION_P),
                SmartDashboard.getNumber("Align/Rotation/kI", Defaults.ROTATION_I),
                SmartDashboard.getNumber("Align/Rotation/kD", Defaults.ROTATION_D));

        // Configure PID controllers
        configurePIDControllers();

        drive = new SwerveRequest.RobotCentric();

        addRequirements(vision, drivetrain);
    }

    private void initializeSmartDashboard() {
        // Translation PID
        SmartDashboard.putNumber("Align/Translation/kP", Defaults.TRANSLATION_P);
        SmartDashboard.putNumber("Align/Translation/kI", Defaults.TRANSLATION_I);
        SmartDashboard.putNumber("Align/Translation/kD", Defaults.TRANSLATION_D);

        // Rotation PID
        SmartDashboard.putNumber("Align/Rotation/kP", Defaults.ROTATION_P);
        SmartDashboard.putNumber("Align/Rotation/kI", Defaults.ROTATION_I);
        SmartDashboard.putNumber("Align/Rotation/kD", Defaults.ROTATION_D);

        // Tolerances and Limits
        SmartDashboard.putNumber("Align/Translation/Tolerance", Defaults.TRANSLATION_TOLERANCE);
        SmartDashboard.putNumber("Align/Rotation/Tolerance", Defaults.ROTATION_TOLERANCE);
        SmartDashboard.putNumber("Align/TargetDistance", Defaults.TARGET_DISTANCE);
        SmartDashboard.putNumber("Align/MaxTranslationSpeed", Defaults.MAX_TRANSLATION_SPEED);
        SmartDashboard.putNumber("Align/MaxRotationSpeed", Defaults.MAX_ROTATION_SPEED);
    }

    private void configurePIDControllers() {
        // Set tolerances
        double translationTolerance = SmartDashboard.getNumber("Align/Translation/Tolerance",
                Defaults.TRANSLATION_TOLERANCE);
        double rotationTolerance = SmartDashboard.getNumber("Align/Rotation/Tolerance",
                Defaults.ROTATION_TOLERANCE);

        xController.setTolerance(translationTolerance);
        yController.setTolerance(translationTolerance);
        rotationController.setTolerance(rotationTolerance);

        // Make rotation continuous
        rotationController.enableContinuousInput(-180, 180);
    }

    private void updatePIDValues() {
        // Update PID values from SmartDashboard
        xController.setPID(
                SmartDashboard.getNumber("Align/Translation/kP", Defaults.TRANSLATION_P),
                SmartDashboard.getNumber("Align/Translation/kI", Defaults.TRANSLATION_I),
                SmartDashboard.getNumber("Align/Translation/kD", Defaults.TRANSLATION_D));

        yController.setPID(
                SmartDashboard.getNumber("Align/Translation/kP", Defaults.TRANSLATION_P),
                SmartDashboard.getNumber("Align/Translation/kI", Defaults.TRANSLATION_I),
                SmartDashboard.getNumber("Align/Translation/kD", Defaults.TRANSLATION_D));

        rotationController.setPID(
                SmartDashboard.getNumber("Align/Rotation/kP", Defaults.ROTATION_P),
                SmartDashboard.getNumber("Align/Rotation/kI", Defaults.ROTATION_I),
                SmartDashboard.getNumber("Align/Rotation/kD", Defaults.ROTATION_D));
    }

    private void updateTelemetry() {
        // Update current state information
        SmartDashboard.putBoolean("Align/HasTarget", vision.hasTarget());
        SmartDashboard.putNumber("Align/CurrentDistance", vision.getVerticalOffset());
        SmartDashboard.putNumber("Align/HorizontalOffset", vision.getHorizontalOffset());

        // Update PID states
        SmartDashboard.putNumber("Align/X/Error", xController.getPositionError());
        SmartDashboard.putNumber("Align/Y/Error", yController.getPositionError());
        SmartDashboard.putNumber("Align/Rotation/Error", rotationController.getPositionError());

        SmartDashboard.putBoolean("Align/X/AtSetpoint", xController.atSetpoint());
        SmartDashboard.putBoolean("Align/Y/AtSetpoint", yController.atSetpoint());
        SmartDashboard.putBoolean("Align/Rotation/AtSetpoint", rotationController.atSetpoint());
    }

    public void configureShuffleboard() {
        ShuffleboardTab alignmentTab = Shuffleboard.getTab("Alignment");

        // PID tuning
        alignmentTab.add("Translation P", Defaults.TRANSLATION_P)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 5));

        // Add similar entries for other PID values

        // Status indicators
        alignmentTab.addBoolean("Has Target", () -> vision.hasTarget())
                .withWidget(BuiltInWidgets.kBooleanBox);

        // Errors and current values
        alignmentTab.addNumber("Distance Error", () -> SmartDashboard.getNumber("Align/X/Error", 0))
                .withWidget(BuiltInWidgets.kGraph);

        // Add new debugging values
        alignmentTab.addNumber("Raw Vertical Offset", () -> vision.getVerticalOffset())
                .withWidget(BuiltInWidgets.kGraph);
        alignmentTab.addNumber("X Speed Command", () -> SmartDashboard.getNumber("Align/XSpeedCommand", 0))
                .withWidget(BuiltInWidgets.kGraph);
        alignmentTab.addBoolean("Limelight Rear Mounted", () -> LIMELIGHT_MOUNTED_ON_BACK)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }

    @Override
    public void initialize() {
        // Reset controllers when command starts
        xController.reset();
        yController.reset();
        rotationController.reset();

        // Update PID values from dashboard
        updatePIDValues();
        configurePIDControllers();

        SmartDashboard.putBoolean("Align/Active", true);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            drivetrain.setControl(drive.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
            updateTelemetry();
            return;
        }

        // Get current robot pose
        Pose2d currentPose = drivetrain.getState().Pose;

        // Get max speeds from dashboard
        double maxTranslationSpeed = SmartDashboard.getNumber("Align/MaxTranslationSpeed",
                Defaults.MAX_TRANSLATION_SPEED);
        double maxRotationSpeed = SmartDashboard.getNumber("Align/MaxRotationSpeed",
                Defaults.MAX_ROTATION_SPEED);

        // Calculate target distance
        double targetDistance = SmartDashboard.getNumber("Align/TargetDistance",
                Defaults.TARGET_DISTANCE);

        // Calculate speeds with limits and apply direction multiplier for rear-mounted
        // Limelight
        double xSpeed = DIRECTION_MULTIPLIER * Math.min(Math.abs(xController.calculate(
                vision.getVerticalOffset(), targetDistance)), maxTranslationSpeed);

        double ySpeed = Math.min(Math.abs(yController.calculate(
                vision.getHorizontalOffset(), 0.0)), maxTranslationSpeed);

        double rotationSpeed = Math.min(Math.abs(rotationController.calculate(
                currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees())), maxRotationSpeed);

        // Add debug values to SmartDashboard
        SmartDashboard.putNumber("Align/RawVerticalOffset", vision.getVerticalOffset());
        SmartDashboard.putNumber("Align/TargetDistance", targetDistance);
        SmartDashboard.putNumber("Align/XSpeedCommand", xSpeed);
        SmartDashboard.putNumber("Align/YSpeedCommand", ySpeed);
        SmartDashboard.putNumber("Align/RotationCommand", rotationSpeed);

        // Apply speeds to swerve drive
        drivetrain.setControl(drive
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed));

        // Update dashboard
        updateTelemetry();
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() &&
                xController.atSetpoint() &&
                yController.atSetpoint() &&
                rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all movement
        drivetrain.setControl(drive
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));

        SmartDashboard.putBoolean("Align/Active", false);
    }
}
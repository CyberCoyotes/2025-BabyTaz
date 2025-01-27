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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.Map;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class VisionCenterCommand_v10 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final CommandXboxController driverController;

    // Updated for CTRE Swerve
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    // Shuffleboard tuning entries
    /* 
    private final GenericEntry pGainEntry = Shuffleboard.getTab("Vision")
            .add("Translation P", Constraints.TRANSLATION_P).getEntry();
    private final GenericEntry rotPGainEntry = Shuffleboard.getTab("Vision")
            .add("Rotation P", Constraints.ROTATION_P).getEntry();
    */
    
    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController rotationController;

    // public VisionImprovedCenterCommand_v5(VisionSubsystem vision,
    // CommandSwerveDrivetrain drivetrain) {

    public VisionCenterCommand_v10(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain,
            CommandXboxController driverController) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.driverController = driverController;

        // Only require vision subsystem since we want to blend driver input
        addRequirements(vision);

        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(VisionConstants.MAX_TRANSLATIONAL_VELOCITY,
                VisionConstants.MAX_TRANSLATIONAL_ACCELERATION);
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                VisionConstants.MAX_ANGULAR_VELOCITY, VisionConstants.MAX_ANGULAR_ACCELERATION);

        strafeController = new ProfiledPIDController(
                VisionConstants.TRANSLATIONAL_kP,
                VisionConstants.TRANSLATIONAL_kI,
                VisionConstants.TRANSLATIONAL_kD,
                translationConstraints,
                0.02);

        rotationController = new ProfiledPIDController(
            VisionConstants.ANGULAR_kP,
            VisionConstants.ANGULAR_kI,
            VisionConstants.ANGULAR_kD,
                rotationConstraints,
                0.02);

        /*
         * // Create Shuffleboard tab for tuning
         * ShuffleboardTab tab = Shuffleboard.getTab("Vision Tuning");
         * // Constraints
         * tab.addNumber("Max Velocity", () -> Constraints.MAX_VELOCITY)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 2.0));
         * 
         * tab.addNumber("Max Accel", () -> Constraints.MAX_ACCELERATION)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 3.0));
         * 
         * tab.addNumber("Max Rotation Velocity", () ->
         * Constraints.MAX_ROTATION_VELOCITY)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 3.0));
         * 
         * tab.addNumber("Max Rotation Accel", () ->
         * Constraints.MAX_ROTATION_ACCELERATION)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 3.0));
         * // Tolerances
         * tab.addNumber("Position Tolerance", () -> Constraints.POSITION_TOLERANCE)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.01, "max", 0.2));
         * tab.addNumber("Rotation Tolerance", () -> Constraints.ROTATION_TOLERANCE)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.01, "max", 0.2));
         * 
         * // Gains
         * tab.addNumber("Translation P", () -> Constraints.TRANSLATION_P)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 0.5));
         * tab.addNumber("Translation I", () -> Constraints.TRANSLATION_I)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 0.5));
         * tab.addNumber("Translation D", () -> Constraints.TRANSLATION_D)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 0.5));
         * tab.addNumber("Rotation P", () -> Constraints.ROTATION_P)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 2.0));
         * tab.addNumber("Rotation I", () -> Constraints.ROTATION_I)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 2.0));
         * tab.addNumber("Rotation D", () -> Constraints.ROTATION_D)
         * .withWidget(BuiltInWidgets.kNumberSlider)
         * .withProperties(Map.of("min", 0.0, "max", 2.0));
         * 
         */

        configurePIDControllers();
        addRequirements(vision, drivetrain);
    }

    private void configurePIDControllers() {
        strafeController.setTolerance(VisionConstants.POSITION_TOLERANCE);
        rotationController.setTolerance(VisionConstants.ANGULAR_TOLERANCE);
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
            // handleDriverInput(); // TODO Test, but should NOT be needed
            return;
        }

        /* 
        TODO Find out the tagId for the vision targets this season.
        */ 
        int tagId = (int) vision.getTagId();
        if (tagId < 1 || tagId > 22) {
            stopMovement();
            return;
        }

        // Calculate vision alignment velocities
        double strafeSpeed = calculateStrafeSpeed();
        double rotationRate = calculateRotationRate();

        // Blend with driver input
        double driverForward = -driverController.getLeftY() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double driverStrafe = -driverController.getLeftX() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double driverRotation = -driverController.getRightX() * VisionConstants.MAX_ANGULAR_VELOCITY;

        // Combine vision and driver inputs
        drivetrain.setControl(drive
                .withVelocityX(driverForward)
                .withVelocityY(strafeSpeed + driverStrafe)
                .withRotationalRate(rotationRate + driverRotation));
        double horizontalOffset = vision.getHorizontalOffset();

        // Scale gains based on distance
        double distanceScale = Math.abs(horizontalOffset) / 10.0;
        distanceScale = Math.max(0.3, distanceScale);

        // Update PID gains from dashboard and scale them
        // strafeController.setP(pGainEntry.getDouble(Constraints.TRANSLATION_P) *
        // distanceScale);
        // rotationController.setP(rotPGainEntry.getDouble(Constraints.ROTATION_P) *
        // distanceScale);

        drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationRate));

        updateTelemetry(horizontalOffset, strafeSpeed, rotationRate);

        
    }

/* Not Sure why this would even be needed...
    private void handleDriverInput() {
        double forward = -driverController.getLeftY() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double strafe = -driverController.getLeftX() * VisionConstants.MAX_TRANSLATIONAL_VELOCITY;
        double rotation = -driverController.getRightX() * VisionConstants.MAX_ANGULAR_VELOCITY;

        drivetrain.setControl(drive
            .withVelocityX(forward)
            .withVelocityY(strafe)
            .withANGULARRate(rotation));
    }
*/
    private double calculateStrafeSpeed() {
        double horizontalOffset = vision.getHorizontalOffset();
        return strafeController.calculate(horizontalOffset, 0);
    }

    private double calculateRotationRate() {
        double horizontalOffset = vision.getHorizontalOffset();
        return rotationController.calculate(Units.degreesToRadians(horizontalOffset), 0);
    }

    private void updateTelemetry(double offset, double strafeSpeed, double rotationRate) {
        SmartDashboard.putNumber("V5l/HorizontalOffset", offset);
        SmartDashboard.putNumber("V5l/StrafeSpeed", strafeSpeed);
        SmartDashboard.putNumber("V5l/RotationRate", rotationRate);
        SmartDashboard.putNumber("V5l/StrateError", strafeController.getPositionError());
        SmartDashboard.putNumber("V5l/RotationError", rotationController.getPositionError());
        SmartDashboard.putBoolean("V5l/AtTarget", isFinished());
        SmartDashboard.putString("V5l/DriveRequestType", drive.DriveRequestType.toString());
        SmartDashboard.putString("V5l/SteerRequestType", drive.SteerRequestType.toString());
        SignalLogger.writeDouble("V5l/HorizontalOffset", offset);
        SignalLogger.writeDouble("V5l/StrafeSpeed", strafeSpeed);
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
    }

}
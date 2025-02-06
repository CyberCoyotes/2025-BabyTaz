package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision18.VisionConstants;
import frc.robot.subsystems.vision18.VisionConstants.PID;
import frc.robot.subsystems.vision18.VisionSubsystem;

import org.littletonrobotics.junction.Logger;

public class AlignToTagCommand18g extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // PID Controllers for both axes
    private final PIDController forwardController;
    private final PIDController lateralController;
    private final PIDController rotationController;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AlignToTagCommand18g(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Forward/back control - using tuned values from ForwardDistanceTest
        forwardController = new PIDController(
                PID.FORWARD_P,
                PID.FORWARD_I,
                PID.FORWARD_D);
        forwardController.setTolerance(PID.FORWARD_TOLERANCE);

        // Left/right control - using tuned values from previous testing
        lateralController = new PIDController(
                PID.LATERAL_P, // 0.4, 0.3, 0.2 best, 0.1
                PID.LATERAL_I,
                PID.LATERAL_D // 0.05, 0.15
        );

        rotationController = new PIDController(
                PID.ROTATION_P,
                PID.ROTATION_I,
                PID.ROTATION_D);
        rotationController.setTolerance(PID.ROTATION_TOLERANCE);

        addRequirements(drivetrain);
        setupLogging();
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            handleNoTarget();
            return;
        }

        // Get Limelight measurements
        double tx = LimelightHelpers.getTX(vision.getName());
        double ty = LimelightHelpers.getTY(vision.getName());
        // double targetSkew = LimelightHelpers.getTargetSkew(vision.getName()); // TODO Get target skew

        // Calculate distances and speeds
        double currentDistance = calculateDistance(ty);

        // Calculate controls for all axes
        double forwardSpeed = forwardController.calculate(currentDistance, VisionConstants.TARGET_DISTANCE);
        double lateralSpeed = lateralController.calculate(tx, 0);
        // double rotationSpeed = rotationController.calculate(targetSkew, 0); // Target 0 skew for perpendicular //

        // Apply speed limits
        forwardSpeed = MathUtil.clamp(forwardSpeed, -VisionConstants.MAX_SPEED, VisionConstants.MAX_SPEED);
        lateralSpeed = MathUtil.clamp(lateralSpeed, -VisionConstants.MAX_SPEED, VisionConstants.MAX_SPEED);
        // rotationSpeed = MathUtil.clamp(rotationSpeed, -VisionConstants.MAX_ANGULAR_SPEED,
                // VisionConstants.MAX_ANGULAR_SPEED);

        // Apply combined control
        drivetrain.setControl(drive
                .withVelocityX(forwardSpeed) // Forward/back
                .withVelocityY(lateralSpeed) // Left/right
                // .withRotationalRate(rotationSpeed)
        );
    }

    private void handleNoTarget() {
        drivetrain.stopDrive();
        Logger.recordOutput("AlignTag/Status", "NO TARGET");
        SmartDashboard.putString("AlignTag/Status", "NO TARGET");
    }

    private double calculateDistance(double ty) {
        double cameraHeight = Units.inchesToMeters(12.5);
        double targetHeight = Units.inchesToMeters(25.5);
        double cameraAngle = 0;

        double angleToTarget = cameraAngle + ty;
        return Math.abs((targetHeight - cameraHeight) / Math.tan(Math.toRadians(angleToTarget))); // Ty is negative,
                                                                                                  // needed to add abs.
    }

    private void logData(double tx, double ty, double distance, double PIDforwardSpeed, double lateralSpeed) {
        // SmartDashboard logging
        SmartDashboard.putNumber("AlignTag/TX", tx);
        SmartDashboard.putNumber("AlignTag/TY", ty);
        SmartDashboard.putNumber("AlignTag/Distance", distance);
        // SmartDashboard.putNumber("AlignTag/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("AlignTag/LateralSpeed", lateralSpeed);
        SmartDashboard.putBoolean("AlignTag/AtForwardSetpoint", forwardController.atSetpoint());
        SmartDashboard.putBoolean("AlignTag/AtLateralSetpoint", lateralController.atSetpoint());

        // AdvantageKit logging
        Logger.recordOutput("AlignCombo/TX", tx);
        Logger.recordOutput("AlignCombo/TY", ty);
        Logger.recordOutput("AlignCombo/Distance", distance);
        // Logger.recordOutput("AlignCombo/ForwardSpeed", forwardSpeed);
        Logger.recordOutput("AlignCombo/LateralSpeed", lateralSpeed);
        Logger.recordOutput("AlignCombo/AtSetpoint",
                forwardController.atSetpoint() && lateralController.atSetpoint());

        // SmartDashboard.putNumber("AlignTag/TargetSkew",
        // LimelightHelpers.getTargetSkew(vision.getName()));
        // SmartDashboard.putNumber("AlignTag/RotationSpeed", rotationSpeed); // FIXME rotationSpeed is not defined
        SmartDashboard.putBoolean("AlignTag/AtRotationSetpoint", rotationController.atSetpoint());

        // Logger.recordOutput("AlignCombo/TargetSkew",
        // LimelightHelpers.getTargetSkew(vision.getName()));
        // Logger.recordOutput("AlignCombo/RotationSpeed", rotationSpeed); // FIXME rotationSpeed is not defined

    }

    private void setupLogging() {
        Logger.recordMetadata("AlignCombo/Description", "Combined alignment to AprilTag");
        Logger.recordMetadata("AlignCombo/TargetDistance",
                String.format("%.2f meters", VisionConstants.TARGET_DISTANCE));
        Logger.recordMetadata("AlignCombo/ForwardPID",
                String.format("kP: %.3f, kI: %.3f, kD: %.3f",
                        forwardController.getP(),
                        forwardController.getI(),
                        forwardController.getD()));
        Logger.recordMetadata("AlignCombo/LateralPID",
                String.format("kP: %.3f, kI: %.3f, kD: %.3f",
                        lateralController.getP(),
                        lateralController.getI(),
                        lateralController.getD()));
    }

    @Override
    public boolean isFinished() {
        return forwardController.atSetpoint() &&
                lateralController.atSetpoint() &&
                rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
        Logger.recordOutput("AlignCombo/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}
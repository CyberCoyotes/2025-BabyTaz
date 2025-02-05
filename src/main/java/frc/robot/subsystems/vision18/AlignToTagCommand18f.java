package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.littletonrobotics.junction.Logger;

public class AlignToTagCommand18f extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;

    // PID Controllers for both axes
    private final PIDController forwardController;
    private final PIDController lateralController;
    
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    
    // Constants
    private static final double TARGET_DISTANCE = 1.0; // meters
    private static final double MAX_SPEED = 1.0; // 50% max speed

    public AlignToTagCommand18f(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Forward/back control - using tuned values from ForwardDistanceTest
        forwardController = new PIDController(
            1.0, 
            0.0, 
            0.0
            );
        forwardController.setTolerance(0.1); // 10cm

        // Left/right control - using tuned values from previous testing
        lateralController = new PIDController(
            0.1, // 0.4, 0.3, 0.2 best, 0.1
            0.0, 
            0.00 // 0.05, 0.15
            );
        lateralController.setTolerance(0.05); // 5cm

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
        
        // Calculate distances and speeds
        double currentDistance = calculateDistance(ty);
        
        // Calculate controls for both axes
        double forwardSpeed = forwardController.calculate(currentDistance, TARGET_DISTANCE);
        double lateralSpeed = lateralController.calculate(tx, 0);
        
        // Apply speed limits
        forwardSpeed = MathUtil.clamp(forwardSpeed, -MAX_SPEED, MAX_SPEED);
        lateralSpeed = MathUtil.clamp(lateralSpeed, -MAX_SPEED, MAX_SPEED);

        // Log data
        logData(tx, ty, currentDistance, forwardSpeed, lateralSpeed);

        // Apply combined control
        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)  // Forward/back
            .withVelocityY(lateralSpeed)  // Left/right
            .withRotationalRate(0));      // No rotation
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
        return Math.abs((targetHeight - cameraHeight) / Math.tan(Math.toRadians(angleToTarget))); // Ty is negative, needed to add abs.
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
    }

    private void setupLogging() {
        Logger.recordMetadata("AlignCombo/Description", "Combined alignment to AprilTag");
        Logger.recordMetadata("AlignCombo/TargetDistance", String.format("%.2f meters", TARGET_DISTANCE));
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
        return forwardController.atSetpoint() && lateralController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
        Logger.recordOutput("AlignCombo/Status", interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}
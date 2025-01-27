package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

// https://claude.ai/chat/768557cd-c3ee-4420-a115-f7c3dc7bb6c8

// This command will:
// Use ProfiledPID control for both rotation and strafing
// Validate AprilTag IDs (1-22)
// Allow live PID tuning via SmartDashboard
// Provide telemetry for debugging
// Use combined rotation and strafe movements to center on the target
// Stop when either no target is visible or the robot is centered and squared up
// You can tune the PID values and speed limits in the Constraints class to match your robot's specific characteristics and requirements.

/*
* horizontalOffset is used for strafing movement only.
* targetYaw (or a similar value from the vision system) drives rotation control.
 */

public class VisionCenterCommand_v2 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric drive;

    // Motion constraints
    private static final class Constraints {
        static final double MAX_VELOCITY = 2.0; // m/s
        static final double MAX_ACCELERATION = 3.0; // m/s²
        static final double MAX_ROTATION_VELOCITY = 2.0; // rad/s
        static final double MAX_ROTATION_ACCELERATION = 3.0; // rad/s²
        
        // Tolerances
        static final double POSITION_TOLERANCE = 0.02; // meters
        static final double ROTATION_TOLERANCE = 0.02; // radians
        
        // PID Gains
        static final double TRANSLATION_P = 1.0;
        static final double TRANSLATION_I = 0.0;
        static final double TRANSLATION_D = 0.0;
        static final double ROTATION_P = 2.0;
        static final double ROTATION_I = 0.0;
        static final double ROTATION_D = 0.1;
    }

    // Profiled PID Controllers
    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController rotationController;

    public VisionCenterCommand_v2(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.RobotCentric();

        TrapezoidProfile.Constraints translationConstraints = 
            new TrapezoidProfile.Constraints(Constraints.MAX_VELOCITY, Constraints.MAX_ACCELERATION);
        
        TrapezoidProfile.Constraints rotationConstraints = 
            new TrapezoidProfile.Constraints(Constraints.MAX_ROTATION_VELOCITY, Constraints.MAX_ROTATION_ACCELERATION);

        strafeController = new ProfiledPIDController(
            Constraints.TRANSLATION_P,
            Constraints.TRANSLATION_I, 
            Constraints.TRANSLATION_D,
            translationConstraints,
            0.02 // 50Hz update rate
        );

        rotationController = new ProfiledPIDController(
            Constraints.ROTATION_P,
            Constraints.ROTATION_I,
            Constraints.ROTATION_D,
            rotationConstraints,
            0.02
        );

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

    // Get vision measurements (in meters or radians)
    double horizontalOffset = vision.getHorizontalOffset(); // For strafing
    // Get the tx value from the vision system
    // double targetYaw = vision.getTargetYaw(); // Angle in radians for rotation // TODO
    
    // double targetYaw = vision.getTargetYaw(); // Angle in radians for rotation // TODO

    // Calculate strafing and rotation speeds independently
    double strafeSpeed = strafeController.calculate(horizontalOffset, 0);
    // double rotationRate = rotationController.calculate(targetYaw, 0);

    // Apply movement
    drivetrain.setControl(drive
        .withVelocityX(0)
        .withVelocityY(strafeSpeed)
        // .withANGULARRate(rotationRate)
        );

    updateTelemetry(horizontalOffset, strafeSpeed); /*, rotationRate targetYaw, strafeSpeed);*/
}

private void updateTelemetry(double offset, /*double yaw,*/ double strafeSpeed) {
    SmartDashboard.putNumber("Vision/HorizontalOffset", offset);
    // SmartDashboard.putNumber("Vision/TargetYaw", yaw);
    SmartDashboard.putNumber("Vision/StrafeSpeed", strafeSpeed);
    // SmartDashboard.putNumber("Vision/RotationRate", rotationRate);
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
    }


    
    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withANGULARRate(0));
    }

}
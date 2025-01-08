package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToTargetCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController alignmentPID;
    private final SwerveRequest.FieldCentric drive;

    // PID constants for rotational alignment
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.01;

    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.alignmentPID = new PIDController(kP, kI, kD);
        this.drive = new SwerveRequest.FieldCentric();
        
        alignmentPID.setTolerance(2.0); // 2 degrees tolerance
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        alignmentPID.reset();
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Calculate rotation speed based on horizontal offset
            double rotationSpeed = alignmentPID.calculate(vision.getHorizontalOffset(), 0);
            
            // Apply rotation while maintaining current translation
            drivetrain.setControl(drive
                .withVelocityX(0)  // No forward/backward movement
                .withVelocityY(0)  // No left/right movement
                .withRotationalRate(rotationSpeed)); // Apply the rotation correction
        }
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || alignmentPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all movement
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
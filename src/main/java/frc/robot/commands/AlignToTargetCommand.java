// AlignToTargetCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class AlignToTargetCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    
    private static final double ROTATION_KP = 0.03;
    private static final double ANGLE_TOLERANCE = 1.0;

    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            double xError = vision.getTargetXAngle();
            double rotationSpeed = -xError * ROTATION_KP;
            
            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed));
                
            SmartDashboard.putNumber("Target Angle Error", xError);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        if (!vision.hasValidTarget()) return true;
        return Math.abs(vision.getTargetXAngle()) < ANGLE_TOLERANCE;
    }
}

package frc.robot.visionV17;

import frc.robot.visionV17.VisionSubsystem17;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

@SuppressWarnings("unused") // Suppresses unused variable warning

public class AlignToTargetCommand17 extends Command {
    private final VisionSubsystem17 vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive;

    // PID Constants for alignment
    private static final double kP = 0.03;
    private static final double ALIGNMENT_TOLERANCE = 2.0; // degrees

    public AlignToTargetCommand17(VisionSubsystem17 vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.drive = new SwerveRequest.FieldCentric()
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Simple proportional control
            double tx = vision.getTX();
            double rotationSpeed = -tx * kP; // Negative because positive tx means target is to the right

            // Apply the rotation while keeping other axes still
            drivetrain.setControl(drive.withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationSpeed));
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() && Math.abs(vision.getTX()) < ALIGNMENT_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}

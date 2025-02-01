package frc.robot.visionV16;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CenterOnTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem16 vision;
    
    // Constants for proportional control
    private static final double kP_Rotation = 0.015; // Tune this value
    private static final double kP_Strafe = 0.015;   // Tune this value
    private static final double MAX_ROTATION_SPEED = 0.5;
    private static final double MAX_STRAFE_SPEED = 0.5;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CenterOnTagCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem16 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        vision.setLeds(true);
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Calculate rotation and strafe speeds using proportional control
            double rotationSpeed = vision.getTX() * kP_Rotation;
            double strafeSpeed = vision.getTX() * kP_Strafe;
            
            // Limit maximum speeds
            rotationSpeed = Math.min(Math.max(rotationSpeed, -MAX_ROTATION_SPEED), MAX_ROTATION_SPEED);
            strafeSpeed = Math.min(Math.max(strafeSpeed, -MAX_STRAFE_SPEED), MAX_STRAFE_SPEED);

            // Apply the speeds to the drivetrain
            drivetrain.setControl(drive
                .withVelocityX(0)  // No forward/backward movement
                .withVelocityY(strafeSpeed)
                .withRotationalRate(rotationSpeed));
        } else {
            // No target found, stop moving
            drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLeds(false);
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return vision.isAlignedToTarget();
    }
}
// AlignToTagCommand.java
package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class AlignToTagCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private final int targetTagID;
    
    private static final double ROTATION_KP = 0.03;
    private static final double DISTANCE_KP = 1.0;
    private static final double TARGET_DISTANCE_METERS = 0.10;
    private static final double DISTANCE_TOLERANCE = 0.02;
    private static final double ANGLE_TOLERANCE = 1.0;
    private static final double MAX_SPEED = 0.5;

    public AlignToTagCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, int targetTagID) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetTagID = targetTagID;
        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        if (vision.hasSpecificTarget(targetTagID)) {
            double xError = vision.getTargetXAngle();
            double currentDistance = vision.getTargetZDistance();
            double distanceError = currentDistance - TARGET_DISTANCE_METERS;

            final double clampedForwardSpeed = Math.max(-MAX_SPEED, 
                Math.min(MAX_SPEED, distanceError * DISTANCE_KP));
            final double clampedRotationSpeed = Math.max(-MAX_SPEED, 
                Math.min(MAX_SPEED, -xError * ROTATION_KP));

            drivetrain.applyRequest(() -> drive
                .withVelocityX(clampedForwardSpeed)
                .withVelocityY(0)
                .withRotationalRate(clampedRotationSpeed));
                
            SmartDashboard.putNumber("Distance to Tag (m)", currentDistance);
            SmartDashboard.putNumber("Distance Error (m)", distanceError);
            SmartDashboard.putNumber("Tag Angle Error", xError);
            SmartDashboard.putNumber("Target Tag ID", targetTagID);
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
        if (!vision.hasSpecificTarget(targetTagID)) return true;
        double distanceError = Math.abs(vision.getTargetZDistance() - TARGET_DISTANCE_METERS);
        double angleError = Math.abs(vision.getTargetXAngle());
        return distanceError < DISTANCE_TOLERANCE && angleError < ANGLE_TOLERANCE;
    }
}
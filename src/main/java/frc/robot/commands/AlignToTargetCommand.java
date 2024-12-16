/*  AlignToTargetCommand.java 
 * This command aligns the robot to a specific vision target by adjusting the robot's orientation and distance from the target.
 * The robot will rotate to face the target.
 *
*/

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

    private static final double ROTATION_KP = 0.1;
    private static final double ANGLE_TOLERANCE = 1.0;
    // TODO
    // Started with 10 frames, but this may need to be adjusted
    // 180 frames is 2 seconds at 90 fps
    private static final int MAX_MISSED_FRAMES = 1000; // Number of frames to keep last known position

    
    private double lastKnownError = 0;
    private int missedFrames = 0;

    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Command Running", true);
        
        if (vision.hasValidTarget()) {
            missedFrames = 0;
            lastKnownError = vision.getTargetXAngle();
            double rotationSpeed = -lastKnownError * ROTATION_KP;
            
            SmartDashboard.putNumber("Rotation Speed", rotationSpeed);
            System.out.println("Target found - x Off: " + lastKnownError + ", speed: " + rotationSpeed);

            drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed));
        } else {
            missedFrames++;
            // Continue using last known error for a few frames
            if (missedFrames < MAX_MISSED_FRAMES) {
                double rotationSpeed = -lastKnownError * ROTATION_KP;
                System.out.println("Using last known error - frames missed: " + missedFrames);
                
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationSpeed));
            } else {
                System.out.println("Too many missed frames, stopping");
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Only finish if we've truly lost the target for an extended period
        return missedFrames >= MAX_MISSED_FRAMES;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Command ending, interrupted: " + interrupted);
        SmartDashboard.putBoolean("Command Running", false);
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
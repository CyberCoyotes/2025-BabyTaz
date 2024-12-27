package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.io.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignSequence extends SequentialCommandGroup {
    
    public AutoAlignSequence(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        addCommands(
            // First make sure we have a valid pose estimate
            new WaitCommand(0.5), // Allow vision processing to stabilize
            
            // Perform coarse alignment using vision
            new AlignToTargetCommand(vision, drivetrain),
            
            // Fine tune position using pose estimation
            new AlignToPoseCommand(vision, drivetrain, targetPose)
        );
    }
}
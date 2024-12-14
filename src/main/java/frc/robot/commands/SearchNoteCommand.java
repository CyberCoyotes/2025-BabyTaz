// SearchNoteCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class SearchNoteCommand extends Command {
    private final NoteDetectionSubsystem noteDetection;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private static final double SEARCH_ROTATION_SPEED = 0.3;
    private double searchStartTime;
    private static final double SEARCH_TIMEOUT = 5.0; // Seconds before giving up

    public SearchNoteCommand(NoteDetectionSubsystem noteDetection, CommandSwerveDrivetrain drivetrain) {
        this.noteDetection = noteDetection;
        this.drivetrain = drivetrain;
        addRequirements(noteDetection, drivetrain);
    }

    @Override
    public void initialize() {
        noteDetection.enableNotePipeline();
        searchStartTime = System.currentTimeMillis() / 1000.0;
        SmartDashboard.putString("Search State", "SEARCHING");
    }

    @Override
    public void execute() {
        if (!noteDetection.hasNote()) {
            // Rotate slowly to search for note
            drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(SEARCH_ROTATION_SPEED));
        }
    }

    @Override
    public void end(boolean interrupted) {
        noteDetection.disableNotePipeline();
        drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        SmartDashboard.putString("Search State", "ENDED");
    }

    @Override
    public boolean isFinished() {
        // End if we find a note or timeout
        double currentTime = System.currentTimeMillis() / 1000.0;
        boolean timedOut = (currentTime - searchStartTime) > SEARCH_TIMEOUT;

        if (timedOut) {
            SmartDashboard.putString("Search State", "TIMED OUT");
        }

        return noteDetection.hasNote() || timedOut;
    }
}

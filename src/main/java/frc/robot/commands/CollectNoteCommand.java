// CollectNoteCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.NoteDetectionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class CollectNoteCommand extends Command {
    private final NoteDetectionSubsystem noteDetection;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    // Move constants here for better encapsulation
    private static final double MAX_ROTATION_SPEED = 0.5;
    private static final double MAX_FORWARD_SPEED = 0.5;
    private static final double ROTATION_KP = 0.03;
    private static final double FORWARD_KP = 1.0;
    private static final double NOTE_COLLECTION_DISTANCE = 0.5;
    private static final double DISTANCE_TOLERANCE = 0.05;
    private static final double ANGLE_TOLERANCE = 2.0;

    // Track state for debugging
    private enum AlignmentState {
        SEARCHING, ALIGNING, APPROACHING, COMPLETE
    }

    private AlignmentState currentState = AlignmentState.SEARCHING;

    public CollectNoteCommand(NoteDetectionSubsystem noteDetection, CommandSwerveDrivetrain drivetrain) {
        this.noteDetection = noteDetection;
        this.drivetrain = drivetrain;
        addRequirements(noteDetection, drivetrain);
    }

    @Override
    public void initialize() {
        noteDetection.enableNotePipeline();
        currentState = AlignmentState.SEARCHING;
        updateTelemetry();
    }

    @Override
    public void execute() {
        if (!noteDetection.hasNote()) {
            currentState = AlignmentState.SEARCHING;
            stopRobot();
            updateTelemetry();
            return;
        }

        double xError = noteDetection.getNoteXAngle();
        double currentDistance = noteDetection.estimateDistance();
        double distanceError = currentDistance - NOTE_COLLECTION_DISTANCE;

        // Update state based on errors
        if (Math.abs(xError) > ANGLE_TOLERANCE) {
            currentState = AlignmentState.ALIGNING;
        } else if (Math.abs(distanceError) > DISTANCE_TOLERANCE) {
            currentState = AlignmentState.APPROACHING;
        } else {
            currentState = AlignmentState.COMPLETE;
        }

        // Calculate speeds based on errors
        double rotationSpeed = Math.max(-MAX_ROTATION_SPEED,
                Math.min(MAX_ROTATION_SPEED, -xError * ROTATION_KP));
        double forwardSpeed = Math.max(-MAX_FORWARD_SPEED,
                Math.min(MAX_FORWARD_SPEED, distanceError * FORWARD_KP));

        // Apply movement
        drivetrain.applyRequest(() -> drive
                .withVelocityX(forwardSpeed)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed));

        updateTelemetry();
    }

    @Override
    public void end(boolean interrupted) {
        noteDetection.disableNotePipeline();
        stopRobot();
        SmartDashboard.putString("Note Collection State", "ENDED");
    }

    @Override
    public boolean isFinished() {
        if (!noteDetection.hasNote())
            return true;
        return currentState == AlignmentState.COMPLETE;
    }

    private void stopRobot() {
        drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    private void updateTelemetry() {
        SmartDashboard.putString("Note Collection State", currentState.toString());
        SmartDashboard.putNumber("Note Distance Error",
                noteDetection.estimateDistance() - NOTE_COLLECTION_DISTANCE);
        SmartDashboard.putNumber("Note Angle Error", noteDetection.getNoteXAngle());
    }
}
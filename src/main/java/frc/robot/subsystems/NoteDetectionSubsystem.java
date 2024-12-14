package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.CollectNoteCommand;
import frc.robot.commands.SearchNoteCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class NoteDetectionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    
    // Constants for note detection and alignment
    private static final int NOTE_PIPELINE = 1; // Separate pipeline for note detection
    private static final double NOTE_COLLECTION_DISTANCE = 0.5; // meters
    private static final double DISTANCE_TOLERANCE = 0.05; // meters
    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    
    // Control constants
    private static final double ROTATION_KP = 0.03;
    private static final double FORWARD_KP = 1.0;
    private static final double MAX_ROTATION_SPEED = 0.5;
    private static final double MAX_FORWARD_SPEED = 0.5;
    
    // Filters for smoothing vision data
    private final MedianFilter xFilter = new MedianFilter(3);
    private final MedianFilter areaFilter = new MedianFilter(3);
    
    // Area to distance conversion constants (needs calibration)
    private static final double AREA_TO_DISTANCE_A = 0.5;
    private static final double AREA_TO_DISTANCE_B = 0.2;
    
    public NoteDetectionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
    }
    
    public Command createCollectNoteCommand() {
        return new CollectNoteCommand(this, drivetrain);
    }

    // Make these public so the command can access them
    public void enableNotePipeline() {
        limelightTable.getEntry("pipeline").setNumber(NOTE_PIPELINE);
        limelightTable.getEntry("ledMode").setNumber(3);
    }
    
    public void disableNotePipeline() {
        limelightTable.getEntry("pipeline").setNumber(0);
        limelightTable.getEntry("ledMode").setNumber(1);
    }
    
    public double estimateDistance() {
        double area = getNoteArea();
        return AREA_TO_DISTANCE_A / (area + AREA_TO_DISTANCE_B);
    }
    
    public boolean hasNote() {
        return tv.getDouble(0.0) == 1.0;
    }
    
    public double getNoteXAngle() {
        return xFilter.calculate(tx.getDouble(0.0));
    }
    
    public double getNoteArea() {
        return areaFilter.calculate(ta.getDouble(0.0));
    }
      
    // Optional: Command to search for a note by rotating
    public Command createSearchNoteCommand() {
        return new SearchNoteCommand(this, drivetrain);
    }

    
    @Override
    public void periodic() {
        if (hasNote()) {
            SmartDashboard.putBoolean("Note Detected", true);
            SmartDashboard.putNumber("Note X Angle", getNoteXAngle());
            SmartDashboard.putNumber("Note Distance", estimateDistance());
        } else {
            SmartDashboard.putBoolean("Note Detected", false);
        }
    }
}
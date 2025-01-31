package frc.robot.visionV15;
// Implement the full motion profiling system
// https://claude.ai/chat/54e71d8f-a11f-4758-92d8-81ec727f8484

import com.ctre.phoenix6.swerve.SwerveRequest;

// Implement the full motion profiling system?

// AlignToAprilTagCommand.java

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.TunableNumber;

public class AlignToAprilTagCommandV15 extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystemV15 vision;
    private boolean stopRequested = false;

    // Tunable PID values
    private final TunableNumber xP = new TunableNumber("Vision/X/P", 0.3);
    private final TunableNumber xI = new TunableNumber("Vision/X/I", 0.001);
    private final TunableNumber xD = new TunableNumber("Vision/X/D", 0.02);
    
    private final TunableNumber yP = new TunableNumber("Vision/Y/P", 0.7);
    private final TunableNumber yI = new TunableNumber("Vision/Y/I", 0.001);
    private final TunableNumber yD = new TunableNumber("Vision/Y/D", 0.02);
    
    private final TunableNumber rotP = new TunableNumber("Vision/Rotation/P", 0.03);
    private final TunableNumber rotI = new TunableNumber("Vision/Rotation/I", 0.0001);
    private final TunableNumber rotD = new TunableNumber("Vision/Rotation/D", 0.002);

    // Motion profiles
    private final TrapezoidProfile xProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(4.0, 3.0)); 
        // 1.0, 0.5 to start
    
        private final TrapezoidProfile yProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(4.0, 3.0));

    private final TrapezoidProfile rotationProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(4.0, 3.0));

    private TrapezoidProfile.State xSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State ySetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State rotationSetpoint = new TrapezoidProfile.State();

    private final PIDController xController = new PIDController(0, 0, 0);
    private final PIDController yController = new PIDController(0, 0, 0);
    private final PIDController rotationController = new PIDController(0, 0, 0);

    // Drive request
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public AlignToAprilTagCommandV15(CommandSwerveDrivetrain drivetrain, VisionSubsystemV15 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        xController.setTolerance(VisionConstantsV15.TRANSLATION_TOLERANCE_METERS);
        yController.setTolerance(VisionConstantsV15.TRANSLATION_TOLERANCE_METERS);       
        rotationController.setTolerance(Math.toRadians(VisionConstantsV15.ANGLE_TOLERANCE_DEGREES));

        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        stopRequested = false;
        vision.setLeds(true);
        updatePIDValues();
        resetProfileStates();
        System.out.println("Starting AprilTag alignment to tag: " + vision.getTagId());
    }

    private void resetProfileStates() {
        xSetpoint = new TrapezoidProfile.State();
        ySetpoint = new TrapezoidProfile.State();
        rotationSetpoint = new TrapezoidProfile.State();
    }

    private void updatePIDValues() {
        xController.setPID(xP.get(), xI.get(), xD.get());
        yController.setPID(yP.get(), yI.get(), yD.get());
        rotationController.setPID(rotP.get(), rotI.get(), rotD.get());
    }

    private boolean isTargetValid() {
        if (!vision.hasTarget()) return false;
        
        if (vision.getTA() < VisionConstantsV15.MIN_TARGET_AREA) return false;
        
        var pose = vision.getTargetPose();
        if (pose == null) return false;
        
        double distance = pose.getTranslation().getNorm();
        if (distance > VisionConstantsV15.MAX_VALID_DISTANCE) return false;
        
    // Instead of ambiguity, check for valid target ID and reasonable tx/ty values
    double tagId = vision.getTagId();
    if (tagId < 1 || tagId > 29) return false;  // Only accept valid April tag IDs

    // Check if tx/ty values are reasonable
    double tx = vision.getTX();
    double ty = vision.getTY();
    if (Double.isNaN(tx) || Double.isNaN(ty)) return false;
    if (Math.abs(tx) > 29.8 || Math.abs(ty) > 24.85) return false; // Standard Limelight FOV limits

    return true;
    }

    @Override
    public void execute() {
        if (!isTargetValid() || stopRequested) {
            // drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
            return;
        }

        var targetPose = vision.getLatencyCompensatedPose();
        if (targetPose.isEmpty()) return;

        // Calculate profile setpoints
        xSetpoint = xProfile.calculate(0.02, xSetpoint, 
            new TrapezoidProfile.State(VisionConstantsV15.TARGET_DISTANCE_METERS, 0));
        ySetpoint = yProfile.calculate(0.02, ySetpoint, 
            new TrapezoidProfile.State(0, 0));
        rotationSetpoint = rotationProfile.calculate(0.02, rotationSetpoint,
            new TrapezoidProfile.State(0, 0));

        // Calculate outputs
        double xSpeed = xController.calculate(
            targetPose.get().getX(), xSetpoint.position);
        double ySpeed = yController.calculate(
            targetPose.get().getY(), ySetpoint.position);
        double rotationRate = rotationController.calculate(
            targetPose.get().getRotation().getZ(), rotationSetpoint.position);

        // Log data
        SmartDashboard.putNumber("Vision/Speeds/X", xSpeed);
        SmartDashboard.putNumber("Vision/Speeds/Y", ySpeed);
        SmartDashboard.putNumber("Vision/Speeds/Rotation", rotationRate);

        // Apply to drivetrain
        drivetrain.setControl(drive
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(rotationRate));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.SwerveDriveBrake());
        vision.setLeds(false);
        System.out.println("Ending AprilTag alignment. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (stopRequested) return true;
        if (!isTargetValid()) return false;

        boolean xAligned = xController.atSetpoint();
        boolean yAligned = yController.atSetpoint();
        boolean rotationAligned = rotationController.atSetpoint();

        SmartDashboard.putBoolean("Vision/Aligned/X", xAligned);
        SmartDashboard.putBoolean("Vision/Aligned/Y", yAligned);
        SmartDashboard.putBoolean("Vision/Aligned/Rotation", rotationAligned);
        
        return xAligned && yAligned && rotationAligned;
    }
}
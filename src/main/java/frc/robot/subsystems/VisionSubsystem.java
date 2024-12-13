package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain; //CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, botpose;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Constants for your Limelight mounting position relative to robot center
    private final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(0.5, 0, 0.5), // Modify these values based on your robot
        new Rotation3d(0, Math.toRadians(30), 0) // Assumes 30 degree mount angle
    );

    // Filters for smoothing vision data
    private final MedianFilter xFilter = new MedianFilter(3);
    private final MedianFilter yFilter = new MedianFilter(3);
    private final MedianFilter rotFilter = new MedianFilter(3);

    // Vision confidence threshold
    private static final double VALID_TARGET_AREA = 0.1; // Minimum target area to consider valid
    private static final double POSE_TRUST_THRESHOLD = 0.9; // Confidence threshold for pose estimation

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        botpose = limelightTable.getEntry("botpose");

        setLimelightConfig();
    }

    private void setLimelightConfig() {
        limelightTable.getEntry("pipeline").setNumber(0);
        limelightTable.getEntry("ledMode").setNumber(1);
        limelightTable.getEntry("camMode").setNumber(0);
        
        // Additional configuration for AprilTags
        limelightTable.getEntry("stream").setNumber(0); // Standard streaming mode
    }

    public boolean hasValidTarget() {
        return tv.getDouble(0.0) == 1.0 && getTargetArea() > VALID_TARGET_AREA;
    }

    public Pose2d getRobotPose() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        if (poseArray.length < 6) {
            return null;
        }
        
        // Apply median filtering to smooth out pose estimates
        double filteredX = xFilter.calculate(poseArray[0]);
        double filteredY = yFilter.calculate(poseArray[1]);
        double filteredRot = rotFilter.calculate(poseArray[5]);
        
        return new Pose2d(
            filteredX,
            filteredY,
            Rotation2d.fromDegrees(filteredRot)
        );
    }

    public void updateOdometryWithVision() {
        if (hasValidTarget()) {
            Pose2d visionPose = getRobotPose();
            if (visionPose != null) {
                // Calculate the distance to target for confidence weighting
                double targetArea = getTargetArea();
                double confidenceMultiplier = Math.min(targetArea / POSE_TRUST_THRESHOLD, 1.0);
                
                // Only update if we're confident enough in the vision measurement
                if (confidenceMultiplier > 0.7) {
                    // Note: You'll need to implement this method in your drivetrain class
                    // or modify to use the appropriate method from CommandSwerveDrivetrain
                    drivetrain.addVisionMeasurement(visionPose, confidenceMultiplier);
                }
            }
        }
    }

    public double getTargetXAngle() {
        return tx.getDouble(0.0);
    }

    public double getTargetYAngle() {
        return ty.getDouble(0.0);
    }

    public double getTargetArea() {
        return ta.getDouble(0.0);
    }

    public Command createAlignToTargetCommand() {
        return new FunctionalCommand(
            // Initialize
            () -> {},
            // Execute
            () -> {
                if (hasValidTarget()) {
                    double xError = getTargetXAngle();
                    double rotationSpeed = -xError * 0.03; // Adjust this multiplier as needed
                    // Note: Modify this to match your drivetrain's drive method signature
                    drivetrain.applyRequest(() -> drive.withRotationalRate(rotationSpeed));
                }
            },
            // End
            interrupted -> drivetrain.applyRequest(() -> drive.withRotationalRate(0)),
            // IsFinished
            () -> Math.abs(getTargetXAngle()) < 1.0,
            // Requirements
            this, drivetrain
        );
    }

    @Override
    public void periodic() {
        if (hasValidTarget()) {
            Pose2d currentPose = getRobotPose();
            if (currentPose != null) {
                SmartDashboard.putNumber("Vision Pose X", currentPose.getX());
                SmartDashboard.putNumber("Vision Pose Y", currentPose.getY());
                SmartDashboard.putNumber("Vision Pose Rotation", currentPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Target Area", getTargetArea());
                SmartDashboard.putNumber("Vision Confidence", 
                    Math.min(getTargetArea() / POSE_TRUST_THRESHOLD, 1.0));
                
                updateOdometryWithVision();
            }
        }
    }
}
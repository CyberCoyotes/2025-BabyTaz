package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;

import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.AlignToTargetCommand;
import frc.robot.commands.AlignToTargetWithDistanceCommand;
import frc.robot.vision.VisionState;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, botpose, tid;
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

    // Constants for distance control
    private static final double TARGET_DISTANCE_METERS = 0.10; // 10 cm target distance
    private static final double DISTANCE_TOLERANCE = 0.02; // 2 cm tolerance
    private static final double DISTANCE_KP = 1.0; // Proportional control constant for distance
    private static final double ROTATION_KP = 0.03; // Proportional control constant for rotation

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        botpose = limelightTable.getEntry("botpose");
        tid = limelightTable.getEntry("tid");

        setLimelightConfig();
    }

    private void setLimelightConfig() {
        limelightTable.getEntry("pipeline").setNumber(0);
        // limelightTable.getEntry("ledMode").setNumber(1);
        limelightTable.getEntry("camMode").setNumber(0);
        limelightTable.getEntry("stream").setNumber(0);
    }

    private VisionState currentState = VisionState.NO_TARGET;

    public VisionState getCurrentState() {
        if (!hasValidTarget()) {
            return VisionState.NO_TARGET;
        }

        int currentTag = getCurrentTagID();
        double xError = getTargetXAngle();

        // Check if robot is aligned with target
        if (Math.abs(xError) < 2.0) { // 2 degrees tolerance
            return VisionState.ALIGNED;
        }

        // Return state based on tag ID
        for (VisionState state : VisionState.values()) {
            if (state.getTagId() == currentTag) {
                return state;
            }
        }

        return VisionState.NO_TARGET;
    }

    public boolean hasValidTarget() {
        double tvValue = tv.getDouble(0.0);
        double taValue = getTargetArea();

        SmartDashboard.putNumber("TV Value", tvValue);
        SmartDashboard.putNumber("TA Value", taValue);

        return tvValue == 1.0; // Remove the area check for now
    }

    public int getCurrentTagID() {
        return (int) tid.getDouble(-1);
    }

    public boolean hasSpecificTarget(int desiredTagID) {
        return hasValidTarget() && getCurrentTagID() == desiredTagID;
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
                Rotation2d.fromDegrees(filteredRot));
    }

    public double getTargetZDistance() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        if (poseArray.length < 6) {
            return 0.0;
        }
        return poseArray[2]; // Z coordinate from botpose
    }

    public void updateOdometryWithVision() {
        if (hasValidTarget()) {
            Pose2d visionPose = getRobotPose();
            if (visionPose != null) {
                double targetArea = getTargetArea();
                double confidenceMultiplier = Math.min(targetArea / POSE_TRUST_THRESHOLD, 1.0);

                if (confidenceMultiplier > 0.7) {
                    drivetrain.seedFieldRelative(visionPose);
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

    // Basic rotation-only alignment
    public Command alignToTarget() {
        System.out.println("Align command created"); // or use SmartDashboard
        SmartDashboard.putString("Vision Command", "Align Started");
        return new AlignToTargetCommand(this, drivetrain);
    }

    // Alignment with distance control for any tag
    public Command alignToTargetWithDistanceCommand() {
        return new AlignToTargetWithDistanceCommand(this, drivetrain);
    }

    // Alignment with distance control for specific tag
    public Command alignToTagWithDistanceCommand(int targetTagID) {
        return new AlignToTagCommand(this, drivetrain, targetTagID);
    }

    private boolean wasConnected = false;

    @Override
    public void periodic() {

        // Connection status
        boolean isConnected = limelightTable.getEntry("tv").isValid();
        if (isConnected != wasConnected) {
            System.out.println("Limelight connection changed: " + isConnected);
            wasConnected = isConnected;
        }
        SmartDashboard.putBoolean("Limelight/Connected", isConnected);

        // Basic target information
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        if (hasValidTarget()) {
            // Target data
            SmartDashboard.putNumber("Limelight/Target/X Angle", getTargetXAngle());
            SmartDashboard.putNumber("Limelight/Target/Area", getTargetArea());
            SmartDashboard.putNumber("Limelight/Target/Tag ID", getCurrentTagID());

            // Pose data (only if we have a valid pose)
            Pose2d currentPose = getRobotPose();
            if (currentPose != null) {
                SmartDashboard.putNumberArray("Limelight/Pose", new double[] {
                        currentPose.getX(),
                        currentPose.getY(),
                        currentPose.getRotation().getDegrees()
                });

                // Vision confidence based on target area
                SmartDashboard.putNumber("Limelight/Confidence",
                        Math.min(getTargetArea() / POSE_TRUST_THRESHOLD, 1.0));

                updateOdometryWithVision();
            }
        }

    } // end of periodic

} // end of class VisionSubsystem
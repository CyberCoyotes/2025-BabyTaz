package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.vision.VisionState;

public class VisionSubsystem extends SubsystemBase {
    // NetworkTables entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tv, tx, ty, ta, botpose, tid;
    private final CommandSwerveDrivetrain drivetrain;
    
    // Constants for vision processing
    private static final double TARGET_DISTANCE_METERS = 0.20; // 20cm target distance
    private static final double DISTANCE_TOLERANCE = 0.02; // 2cm tolerance
    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double POSE_TRUST_THRESHOLD = 0.9;

    // Filters for smoothing vision data
    private final MedianFilter xFilter = new MedianFilter(3);
    private final MedianFilter yFilter = new MedianFilter(3);
    private final MedianFilter areaFilter = new MedianFilter(3);

    // State tracking
    private boolean wasConnected = false;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize NetworkTables
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
        limelightTable.getEntry("camMode").setNumber(0);
        limelightTable.getEntry("stream").setNumber(0);
    }

    // Basic target detection methods
    public boolean hasValidTarget() {
        return tv.getDouble(0.0) == 1.0;
    }

    public int getCurrentTagID() {
        return (int) tid.getDouble(-1);
    }

    public boolean hasSpecificTarget(int desiredTagID) {
        return hasValidTarget() && getCurrentTagID() == desiredTagID;
    }

    // Distance and angle methods
    public boolean isTargetInRange() {
        if (!hasValidTarget()) return false;
        return Math.abs(getTargetZDistance() - TARGET_DISTANCE_METERS) < DISTANCE_TOLERANCE;
    }

    public double getDistanceError() {
        if (!hasValidTarget()) return 0.0;
        return getTargetZDistance() - TARGET_DISTANCE_METERS;
    }

    public double getTargetXAngle() {
        return xFilter.calculate(tx.getDouble(0.0));
    }

    public double getTargetYAngle() {
        return yFilter.calculate(ty.getDouble(0.0));
    }

    public double getTargetArea() {
        return areaFilter.calculate(ta.getDouble(0.0));
    }

    public double getTargetZDistance() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        return poseArray.length >= 6 ? poseArray[2] : 0.0;
    }

    // State tracking
    public VisionState getCurrentState() {
        if (!hasValidTarget()) {
            return VisionState.NO_TARGET;
        }
        
        int currentTag = getCurrentTagID();
        boolean inRange = isTargetInRange();
        
        switch (currentTag) {
            case 1:
                return inRange ? VisionState.TARGET_1_CLOSE : VisionState.TARGET_1;
            case 2:
                return inRange ? VisionState.TARGET_2_CLOSE : VisionState.TARGET_2;
            case 3:
                return inRange ? VisionState.TARGET_3_CLOSE : VisionState.TARGET_3;
            case 4:
                return inRange ? VisionState.TARGET_4_CLOSE : VisionState.TARGET_4;
            default:
                return VisionState.NO_TARGET;
        }
    }

    // Pose estimation
    public Pose2d getRobotPose() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        if (poseArray.length < 6) {
            return null;
        }

        return new Pose2d(
            xFilter.calculate(poseArray[0]),
            yFilter.calculate(poseArray[1]),
            Rotation2d.fromDegrees(poseArray[5])
        );
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

    // Constants access methods
    public double getTargetRange() {
        return TARGET_DISTANCE_METERS;
    }

    public double getDistanceTolerance() {
        return DISTANCE_TOLERANCE;
    }

    @Override
    public void periodic() {
        // Connection status
        boolean isConnected = tv.isValid();
        if (isConnected != wasConnected) {
            System.out.println("Limelight connection changed: " + isConnected);
            wasConnected = isConnected;
        }
        SmartDashboard.putBoolean("Limelight/Connected", isConnected);

        // Vision processing status
        SmartDashboard.putBoolean("Limelight/Has Target", hasValidTarget());
        if (hasValidTarget()) {
            VisionState currentState = getCurrentState();
            SmartDashboard.putString("Limelight/State", currentState.getDescription());
            SmartDashboard.putNumber("Limelight/Target/X Angle", getTargetXAngle());
            SmartDashboard.putNumber("Limelight/Target/Distance", getTargetZDistance());
            SmartDashboard.putNumber("Limelight/Target/Distance Error", getDistanceError());
            SmartDashboard.putBoolean("Limelight/Target/In Range", isTargetInRange());
            SmartDashboard.putNumber("Limelight/Target/Tag ID", getCurrentTagID());
            
            // Pose estimation
            Pose2d currentPose = getRobotPose();
            if (currentPose != null) {
                SmartDashboard.putNumberArray("Limelight/Pose", new double[] {
                    currentPose.getX(),
                    currentPose.getY(),
                    currentPose.getRotation().getDegrees()
                });
                updateOdometryWithVision();
            }
        }
    }
}
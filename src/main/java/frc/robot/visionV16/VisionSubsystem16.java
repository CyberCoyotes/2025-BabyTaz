package frc.robot.visionV16;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.visionV16.FieldElementLocation;

public class VisionSubsystem16 extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final LEDSubsystem ledSubsystem;
    private final CommandSwerveDrivetrain drivetrain;
    private String limelightName;

    public VisionSubsystem16(String limelightName, CommandSwerveDrivetrain drivetrain, LEDSubsystem ledSubsystem) {
        this.limelightName = limelightName;
        this.drivetrain = drivetrain;
        this.ledSubsystem = ledSubsystem;
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

        // By default, turn off LEDs
        setLeds(false);
    }

    // TODO Move to VisionConstants
    String elementType = FieldElementLocation.getElementType(0);

    @Override
    public void periodic() {
        // Log vision data to SmartDashboard
        SmartDashboard.putBoolean("V16 Has Target", hasTarget());
        SmartDashboard.putNumber("V16 Target ID", getTagId());
        SmartDashboard.putNumber("V16 TX", getTX());
        SmartDashboard.putNumber("V16 TY", getTY());
        SmartDashboard.putNumber("V16 TA", getTA());
        SmartDashboard.putNumber("V16 Distance (m)", getDistanceToTargetMeters());
        SmartDashboard.putString("Reef Type", elementType);
    }

    // Basic target detection
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    // Get AprilTag ID (-1 if no tag detected)
    public double getTagId() {
        return limelightTable.getEntry("tid").getDouble(-1);
    }

    // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
    public double getTX() {
        // TODO added LIMELIGHT DIRECTION to test the mounting pose as front facing or back facing.
        return limelightTable.getEntry("tx").getDouble(0.0) * (VisionConstants.LIMELIGHT_DIRECTION);

    }

    // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
    public double getTY() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    // Target Area (0% of image to 100% of image)
    public double getTA() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    // Control Limelight LED state
    public void setLeds(boolean enabled) {
        limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1);
    }

    // Additional helper methods for specific game tasks
    public boolean isAlignedToTarget() {
        return hasTarget() && Math.abs(getTX()) < 2.0; // Within 2 degrees
    }

        // Add this method to calculate distance
        public double getDistanceToTargetMeters() {
            if (!hasTarget()) {
                return 0.0;
            }
    
            // Get the vertical angle to target from Limelight
            double targetAngleVertical = getTY();
    
            // Calculate distance using trigonometry
            double angleToTargetRadians = Math.toRadians(VisionConstants16.LIMELIGHT_MOUNT_ANGLE_DEGREES + targetAngleVertical);
            double distanceMeters = (VisionConstants16.REEF_TARGET_HEIGHT - VisionConstants16.LIMELIGHT_MOUNT_HEIGHT_METERS) 
                                   / Math.tan(angleToTargetRadians);
    
            return distanceMeters;
        }

    }
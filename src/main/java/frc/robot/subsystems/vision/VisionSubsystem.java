package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDState;
import frc.robot.subsystems.led.LEDSubsystem;


public class VisionSubsystem extends SubsystemBase {
    private final String tableName;
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDSubsystem leds;
    
    // NetworkTable entries
    private final NetworkTableEntry tv; // Whether there are valid targets
    private final NetworkTableEntry tx; // Horizontal offset
    private final NetworkTableEntry ty; // Vertical offset
    private final NetworkTableEntry ta; // Target area
    private final NetworkTableEntry tid; // AprilTag ID
    
    // Vision processing constants
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double VALID_TARGET_AREA = 0.1; // % of image
    
    private VisionState currentState = VisionState.NO_TARGET;
    private boolean ledsEnabled = true;

    public VisionSubsystem(String tableName, CommandSwerveDrivetrain drivetrain, LEDSubsystem leds) {
        this.tableName = tableName;
        this.drivetrain = drivetrain;
        this.leds = leds;
        
        // Initialize NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(tableName);
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        tid = limelightTable.getEntry("tid");
        
        // Configure Limelight
        configureLimelight();
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline
        limelightTable.getEntry("pipeline").setNumber(0);
        setLeds(true);
    }

    @Override
    public void periodic() {
        updateVisionState();
        updateLEDs();
        logData();
    }

    private void updateVisionState() {
        boolean hasTarget = tv.getDouble(0.0) > 0.5;
        double horizontalOffset = tx.getDouble(0.0);
        double area = ta.getDouble(0.0);

        if (!hasTarget || area < VALID_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
    }

    public void setLeds(boolean enabled) {
        ledsEnabled = enabled;
        limelightTable.getEntry("ledMode").setNumber(enabled ? 3 : 1); // 3=force on, 1=force off
    }

    private void updateLEDs() {
        if (leds != null) {
            switch (currentState) {
                case TARGET_LOCKED:
                    leds.setState(LEDState.TARGET_LOCKED);
                    break;
                case TARGET_VISIBLE:
                    leds.setState(LEDState.TARGET_VISIBLE);
                    break;
                case NO_TARGET:
                default:
                    leds.setState(LEDState.NO_TARGET);
                    break;
            }
        }
    }

    private void logData() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", tid.getDouble(0));
        SmartDashboard.putNumber("Vision/TX", tx.getDouble(0));
        SmartDashboard.putNumber("Vision/TY", ty.getDouble(0));
        SmartDashboard.putNumber("Vision/TA", ta.getDouble(0));
    }

    // Getter methods for use in commands
    public VisionState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        return currentState != VisionState.NO_TARGET;
    }

    public double getHorizontalOffset() {
        return tx.getDouble(0.0);
    }

    public double getVerticalOffset() {
        return ty.getDouble(0.0);
    }

    public int getTagId() {
        return (int) tid.getDouble(0);
    }
}
package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.led.LEDState;
import frc.robot.subsystems.led.LEDSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final LEDSubsystem leds;
    private VisionState currentState = VisionState.NO_TARGET;

    // Vision processing constants 
    private static final double TARGET_LOCK_THRESHOLD = 2.0; // Degrees
    private static final double MIN_TARGET_AREA = 0.1; // % of image

    public VisionSubsystem(String limelightName, LEDSubsystem leds) {
        this.limelightName = limelightName;
        this.leds = leds;
        configureLimelight();
    }

    private void configureLimelight() {
        // Set to AprilTag pipeline by default
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setLEDMode_ForceOn(limelightName);
    }

    @Override
    public void periodic() {
        updateVisionState();
        updateLEDs();
        logTelemetry();
    }

    private void updateVisionState() {
        boolean hasTarget = LimelightHelpers.getTV(limelightName);
        double horizontalOffset = getHorizontalOffset();
        double targetArea = LimelightHelpers.getTA(limelightName);

        if (!hasTarget || targetArea < MIN_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= TARGET_LOCK_THRESHOLD) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }

        // Add to VisionSubsystem periodic():
        boolean targetValid = LimelightHelpers.getTV(limelightName);
        // System.out.println("Target Valid: " + targetValid);
        System.out.println("Current State: " + getState());
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
                    leds.setState(LEDState.NO_TARGET);
                    break;
            }
        }
    }

    public VisionState getState() {
        return currentState;
    }

    public boolean hasTarget() {
        boolean valid = LimelightHelpers.getTV(limelightName);
        return valid;

    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName) * (VisionConstants.LIMELIGHT_DIRECTION);
    }

    public double getVerticalOffset() {
        return LimelightHelpers.getTY(limelightName);
    }

    public int getTagId() {
        return (int) LimelightHelpers.getFiducialID(limelightName);
    }

    public boolean isTagValid(int tagId) {
        return tagId >= VisionConstants.MIN_VALID_TAG && 
               tagId <= VisionConstants.MAX_VALID_TAG;
    }

    private void logTelemetry() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/TagID", getTagId());
        SmartDashboard.putNumber("Vision/HorizontalOffset", getHorizontalOffset());
        SmartDashboard.putNumber("Vision/VerticalOffset", getVerticalOffset());
        SmartDashboard.putNumber("Vision/TargetArea", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());
    }

    /*
    public enum VisionState {
        NO_TARGET,
        TARGET_VISIBLE,
        TARGET_LOCKED
    } 
    */
}
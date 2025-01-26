package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.led.LEDState;
import frc.robot.subsystems.led.LEDSubsystem;

public class VisionSubsystem extends SubsystemBase {
    private final String limelightName;
    private final LEDSubsystem leds;
    private VisionState currentState = VisionState.NO_TARGET;

    public VisionSubsystem(String limelightName, LEDSubsystem leds) {
        this.limelightName = limelightName;
        this.leds = leds;
        configureLimelight();
    }

    private void configureLimelight() {
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        // pipeline index 0 will be used for vision processing of AprilTags

        // LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        LimelightHelpers.setLEDMode_ForceOff(limelightName);
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

        if (!hasTarget || targetArea < VisionConstants.MIN_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else if (Math.abs(horizontalOffset) <= VisionConstants.POSITION_TOLERANCE /*TODO Add a ROTATIONAL_TOL check? */) {
            currentState = VisionState.TARGET_LOCKED;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
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
        return currentState != VisionState.NO_TARGET;
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(limelightName) * 
               (VisionConstants.LIMELIGHT_DIRECTION);
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

    public double getDistanceToTarget() { // TODO
        double targetHeightMeters = 0.61; // Adjust based on AprilTag height
        double cameraHeightMeters = 0.5;  // Adjust based on camera mount
        double cameraPitchDegrees = 0.0;  // Adjust based on camera angle
        
        double targetPitchDegrees = getVerticalOffset();
        double angleRadians = Math.toRadians(cameraPitchDegrees + targetPitchDegrees);
        
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleRadians);
    }
    
    

    private void logTelemetry() {
        SmartDashboard.putString("VSub/State", currentState.toString());
        SmartDashboard.putNumber("VSub/TagID", getTagId());
        SmartDashboard.putNumber("VSub/HorizontalOffset", getHorizontalOffset());
        SmartDashboard.putNumber("VSub/VerticalOffset", getVerticalOffset());
        SmartDashboard.putNumber("VSub/TargetArea", LimelightHelpers.getTA(limelightName));
        SmartDashboard.putBoolean("VSub/HasTarget", hasTarget());
    }

}
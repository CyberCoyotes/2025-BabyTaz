// LimelightVisionIO.java
package frc.robot.io;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.io.led.LEDMode;


public class LimelightVisionIO implements VisionIO {
    private final String limelightName;
    private VisionState currentState = VisionState.NO_TARGET;
    private double lastValidTargetTimestamp = 0;

    private double ALIGNMENT_TOLERANCE_X = 3.0;
    private double ALIGNMENT_TOLERANCE_Y = 3.0;

    public LimelightVisionIO(String name) {
        this.limelightName = name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update basic targeting data
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);
        inputs.targetXOffset = LimelightHelpers.getTX(limelightName);
        inputs.targetYOffset = LimelightHelpers.getTY(limelightName);
        inputs.targetArea = LimelightHelpers.getTA(limelightName);
        inputs.latencyMillis = LimelightHelpers.getLatency_Pipeline(limelightName);
        inputs.pipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex(limelightName);
        
        // Update state
        updateVisionState(inputs);
        
        // Update pose data if available
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (poseEstimate != null) {
            inputs.robotPose = poseEstimate.pose;
            inputs.visibleTagCount = poseEstimate.tagCount;
            inputs.averageTagDistance = poseEstimate.avgTagDist;
        }
    }

    private void updateVisionState(VisionIOInputs inputs) {
        if (!inputs.hasTarget) {
            inputs.state = VisionState.NO_TARGET;
            return;
        }

        inputs.lastValidTimestamp = Timer.getFPGATimestamp();
        
        // Check if we're aligned with the target
        boolean isAligned = Math.abs(inputs.targetXOffset) < ALIGNMENT_TOLERANCE_X &&
                          Math.abs(inputs.targetYOffset) < ALIGNMENT_TOLERANCE_Y;

        inputs.state = isAligned ? VisionState.TARGET_ALIGNED : VisionState.TARGET_DETECTED;
    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(limelightName, index);
    }

    @Override
    public void setLEDMode(LEDMode mode) {
        switch (mode) {
            case PIPELINE:
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
                break;
            case OFF:
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
                break;
            case BLINK:
                LimelightHelpers.setLEDMode_ForceBlink(limelightName);
                break;
            case ON:
                LimelightHelpers.setLEDMode_ForceOn(limelightName);
                break;
        }
    }
}
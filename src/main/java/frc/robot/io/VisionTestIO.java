// VisionTestIO.java
package frc.robot.io;

public class VisionTestIO implements VisionIO {
    private VisionState simulatedState = VisionState.NO_TARGET;
    private boolean hasTarget = false;
    private double targetX = 0.0;
    private double targetY = 0.0;

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.hasTarget = hasTarget;
        inputs.targetXOffset = targetX;
        inputs.targetYOffset = targetY;
        inputs.state = simulatedState;
    }

    // Methods for testing
    public void setSimulatedTarget(boolean hasTarget, double x, double y) {
        this.hasTarget = hasTarget;
        this.targetX = x;
        this.targetY = y;
    }

    public void setSimulatedState(VisionState state) {
        this.simulatedState = state;
    }
}
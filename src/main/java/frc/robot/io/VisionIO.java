package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.io.led.LEDMode;

public interface VisionIO {
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public double targetXOffset = 0.0;
        public double targetYOffset = 0.0;
        public double targetArea = 0.0;
        public double latencyMillis = 0.0;
        public int pipelineIndex = 0;
        public VisionState state = VisionState.NO_TARGET;
        public Pose2d robotPose = new Pose2d();
        public double lastValidTimestamp = 0.0;
        public int visibleTagCount = 0;
        public double averageTagDistance = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {}
    
    /** Sets the pipeline index. */
    public default void setPipeline(int index) {}
    
    /** Sets the LED mode. */
    public default void setLEDMode(LEDMode mode) {}
}

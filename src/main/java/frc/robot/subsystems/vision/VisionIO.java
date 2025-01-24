/* 
 * Structure inspired by FRC 2910 
 * https://github.com/FRCTeam2910/2024CompetitionRobot-Public/tree/main/src/main/java/frc/robot/subsystems
*/
package frc.robot.subsystems.vision;

public interface VisionIO {
    public default void updateInputs(VisionIOInputs inputs) {}
    // public default void setLeds(boolean on) {}

    public static class VisionIOInputs {
        // Timestamp data
        public double lastTimeStamp = 0.0;
        
        // Target data
        public boolean hasTargets = false;
        public double verticalAngleRadians = 0.0;
        public double horizontalAngleRadians = 0.0;
        
        // AprilTag specific data  
        public double[] botpose = new double[6];
        public int tagId = -1;
        
        // Latency data
        public double pipelineLatency = 0.0;
        public double captureLatency = 0.0;
    }
}
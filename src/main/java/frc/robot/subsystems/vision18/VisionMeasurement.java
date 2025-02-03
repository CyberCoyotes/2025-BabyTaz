package frc.robot.subsystems.vision18;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision18.VisionConstants18;

public class VisionMeasurement {
    private final Pose2d pose;
    private final double timestamp;
    private final double latency;
    private final double ambiguity;
    private final int numTags;
    private final double avgDistance;
    private final List<Integer> tagIds;

    public VisionMeasurement(Pose2d pose, double timestamp, double latency, 
                           double ambiguity, int numTags, double avgDistance,
                           List<Integer> tagIds) {
        this.pose = pose;
        this.timestamp = timestamp;
        this.latency = latency;
        this.ambiguity = ambiguity;
        this.numTags = numTags;
        this.avgDistance = avgDistance;
        this.tagIds = tagIds;
    }

    // Getters...
    
    public boolean isValid() {
        return numTags > 0 && 
               latency < VisionConstants18.MAX_POSE_LATENCY &&
               ambiguity < VisionConstants18.MAX_POSE_AMBIGUITY &&
               avgDistance < VisionConstants18.MAX_RANGE;
    }

    public Pose2d getPose() {
        return pose;
    }
    
    public double getTimestamp() {
        return timestamp;
    }
    
    public double getLatency() {
        return latency;
    }
    
    public double getAvgDistance() {
        return avgDistance;
    }
    
    public int getNumTags() {
        return numTags;
    }
    
    public List<Integer> getTagIds() {
        return tagIds;
    }
    
    /*
    public double getAmbiguity() {
        return ambiguity;
    }    
    */


}
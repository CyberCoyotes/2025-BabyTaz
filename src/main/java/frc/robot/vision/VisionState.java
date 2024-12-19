// Create new file: src/main/java/frc/robot/vision/VisionState.java
package frc.robot.vision;

public enum VisionState {
    NO_TARGET("No Valid Target", -1),
    TARGET_1("Tag 1 Detected", 1),
    TARGET_2("Tag 2 Detected", 2), 
    TARGET_3("Tag 3 Detected", 3),
    TARGET_4("Tag 4 Detected", 4),
    ALIGNED("Target Aligned", -2);

    private final String description;
    private final int tagId;

    VisionState(String description, int tagId) {
        this.description = description;
        this.tagId = tagId;
    }

    public String getDescription() {
        return description;
    }

    public int getTagId() {
        return tagId;
    }
}
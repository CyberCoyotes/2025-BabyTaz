package frc.robot.experimental;

// Updated VisionState.java
public enum VisionState {
    NO_TARGET("No Valid Target", -1, false),
    TARGET_1("Tag 1 Detected", 1, false),
    TARGET_2("Tag 2 Detected", 2, false), 
    TARGET_3("Tag 3 Detected", 3, false),
    TARGET_4("Tag 4 Detected", 4, false),
    TARGET_1_CLOSE("Tag 1 In Range", 1, true),
    TARGET_2_CLOSE("Tag 2 In Range", 2, true),
    TARGET_3_CLOSE("Tag 3 In Range", 3, true),
    TARGET_4_CLOSE("Tag 4 In Range", 4, true);

    private final String description;
    private final int tagId;
    private final boolean inRange;

    VisionState(String description, int tagId, boolean inRange) {
        this.description = description;
        this.tagId = tagId;
        this.inRange = inRange;
    }

    public String getDescription() { return description; }
    public int getTagId() { return tagId; }
    public boolean isInRange() { return inRange; }

}
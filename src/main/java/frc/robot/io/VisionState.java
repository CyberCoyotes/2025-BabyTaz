package frc.robot.io;

public enum VisionState {
    NO_TARGET("No Target"),
    TARGET_DETECTED("Target Detected"),
    TARGET_ALIGNED("Target Aligned");

    private final String description;

    VisionState(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }
}


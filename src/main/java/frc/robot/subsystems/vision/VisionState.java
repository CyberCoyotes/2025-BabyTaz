package frc.robot.subsystems.vision;

public enum VisionState {
    NO_TARGET(0),
    TARGET_VISIBLE(1),
    TARGET_LOCKED(2);

    private final int value;
    
    VisionState(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
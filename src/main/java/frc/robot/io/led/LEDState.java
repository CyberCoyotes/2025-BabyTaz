package frc.robot.io.led;

public enum LEDState {
    OFF(0, 0, 0),
    ENABLED(0, 255, 0),
    DISABLED(255, 20, 0),
    AUTONOMOUS(0, 0, 255),
    TARGET_VISIBLE(255, 255, 0),
    TARGET_LOCKED(0, 255, 0),
    INTAKING(0, 255, 255),
    SCORING(255, 105, 180),
    ERROR(255, 0, 0);

    public final int r, g, b;

    LEDState(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}

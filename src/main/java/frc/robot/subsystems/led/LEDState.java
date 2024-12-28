package frc.robot.subsystems.led;

public enum LEDState {
    OFF(0, 0, 0),                    // Black (completely off)
    ENABLED(0, 255, 0),             // Pure Green
    DISABLED(255, 20, 0),           // Dark Orange-Red
    AUTONOMOUS(0, 0, 255),          // Pure Blue
    TARGET_VISIBLE(255, 255, 0),    // Bright Yellow
    TARGET_LOCKED(0, 255, 0),       // Pure Green
    INTAKING(0, 255, 255),         // Cyan/Aqua
    SCORING(255, 105, 180),        // Hot Pink
    ERROR(255, 0, 0);              // Pure Red

    public final int r, g, b;

    LEDState(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
// frc/robot/io/led/LEDTestIO.java
package frc.robot.io.led;

public class LEDTestIO implements LEDIO {
    private LEDState currentState = LEDState.OFF;
    private int r = 0, g = 0, b = 0;

    @Override
    public void updateInputs(LEDIOInputs inputs) {
        inputs.currentState = currentState;
        inputs.totalLEDs = 8; // Simulate 8 LEDs for testing
        inputs.vbat = 12.0;   // Simulate 12V battery
        inputs.v5 = 4.9;      // Simulate 5V rail
    }

    @Override
    public void setState(LEDState state) {
        this.currentState = state;
        this.r = state.r;
        this.g = state.g;
        this.b = state.b;
    }

    @Override
    public void setColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
// frc/robot/io/led/LEDIO.java
package frc.robot.io.led;

import com.ctre.phoenix.led.Animation;

public interface LEDIO {
    public static class LEDIOInputs {
        public LEDState currentState = LEDState.OFF;
        public int totalLEDs = 0;
        public double vbat = 0.0;
        public double v5 = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(LEDIOInputs inputs) {}
    
    /** Sets the LED state */
    public default void setState(LEDState state) {}
    
    /** Sets a specific LED color */
    public default void setColor(int r, int g, int b) {}
    
    /** Sets a section of LEDs to a specific color */
    public default void setLEDsSection(int startIndex, int count, int r, int g, int b) {}
    
    /** Starts an animation */
    public default void setAnimation(Animation animation) {}
    
    /** Clear all animations and return to solid colors */
    public default void clearAnimation() {}
}

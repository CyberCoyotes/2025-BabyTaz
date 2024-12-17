package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;
    private LEDState currentState = LEDState.OFF;
    private Animation currentAnimation = null;

    // LED strip configuration
    private static final int LED_COUNT = 40;  // Total number of LEDs
    private static final double DEFAULT_BRIGHTNESS = 0.90;
    
    public enum LEDState {
        OFF,
        READY,
        ERROR,
        AUTONOMOUS,
        TELEOP,
        TARGET_VISIBLE,
        TARGET_LOCKED,
        INTAKE_READY,
        NOTE_HELD,
        SHOOTING
    }

    public enum LEDColor {
        RED(255, 0, 0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0),
        PURPLE(255, 0, 255),
        WHITE(255, 255, 255),
        ORANGE(255, 165, 0),
        CYAN(0, 255, 255),
        GOLD(255, 215, 0),
        OFF(0, 0, 0);

        private final int r, g, b;

        LEDColor(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public LEDSubsystem(int candleID) {
        try {
            candle = new CANdle(candleID);
            configureCANdle();
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize CANdle: " + e.getMessage(), e.getStackTrace());
            throw e;
        }
    }

    private void configureCANdle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = DEFAULT_BRIGHTNESS;
        config.stripType = CANdle.LEDStripType.RGB;
        config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        
        candle.configAllSettings(config);
        setOff(); // Start with LEDs off
    }

    @Override
    public void periodic() {
        // Update LEDs based on robot state
        updateLEDsBasedOnState();
    }

    private void updateLEDsBasedOnState() {
        switch (currentState) {
            case AUTONOMOUS:
                setColor(LEDColor.GOLD);
                break;
            case TELEOP:
                if (DriverStation.isEnabled()) {
                    setColor(LEDColor.GREEN);
                } else {
                    setColor(LEDColor.BLUE);
                }
                break;
            case ERROR:
                // Strobe red for errors
                if (currentAnimation == null || !(currentAnimation instanceof StrobeAnimation)) {
                    currentAnimation = new StrobeAnimation(255, 0, 0, 0, 0.5, LED_COUNT);
                    candle.animate(currentAnimation);
                }
                break;
            // Add other states as needed
        }
    }

    public void setState(LEDState state) {
        if (state != currentState) {
            currentState = state;
            stopAnimation(); // Clear any running animation
            updateLEDsBasedOnState();
        }
    }

    public void setColor(LEDColor color) {
        stopAnimation();
        try {
            candle.setLEDs(color.r, color.g, color.b);
        } catch (Exception e) {
            DriverStation.reportError("Failed to set LED color: " + e.getMessage(), e.getStackTrace());
        }
    }

    public void startRainbow() {
        stopAnimation();
        currentAnimation = new RainbowAnimation(1, 0.5, LED_COUNT);
        candle.animate(currentAnimation);
    }

    public void strobeColor(LEDColor color) {
        stopAnimation();
        currentAnimation = new StrobeAnimation(color.r, color.g, color.b, 0, 0.5, LED_COUNT);
        candle.animate(currentAnimation);
    }

    public void stopAnimation() {
        if (currentAnimation != null) {
            candle.clearAnimation(0);
            currentAnimation = null;
        }
    }

    public void setOff() {
        setState(LEDState.OFF);
        setColor(LEDColor.OFF);
    }

    public LEDState getCurrentState() {
        return currentState;
    }

    // Example usage in robot code:
    // leds.setState(LEDState.TARGET_VISIBLE);  // When target is seen
    // leds.setState(LEDState.TARGET_LOCKED);   // When aligned
    // leds.setState(LEDState.NOTE_HELD);       // When note is in robot
    // leds.strobeColor(LEDColor.GREEN);        // For successful actions
}
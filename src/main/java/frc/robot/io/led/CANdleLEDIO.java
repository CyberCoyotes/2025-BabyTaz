// frc/robot/io/led/CANdleLEDIO.java
package frc.robot.io.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.Timer;

public class CANdleLEDIO implements LEDIO {
    private final CANdle candle;
    private LEDState currentState = LEDState.OFF;
    private final Timer configWaitTimer = new Timer();
    private static final double CONFIG_WAIT_TIME = 0.1; // seconds to wait between configs
    private static final int NUM_LEDS = 8; // Set this to your actual number of LEDs

    public CANdleLEDIO(int candleID) {
        candle = new CANdle(candleID);
        configureCANdle();
    }

    private void configureCANdle() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 0.8;    // 80% brightness
        config.vBatOutputMode = true;      // Enable vBat monitoring
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;     // Don't disable on loss of signal
        config.stripType = CANdle.LEDStripType.RGB; // Set your LED strip type
        config.numLed = NUM_LEDS; // Set the number of LEDs
        candle.configAllSettings(config);
    }

    @Override
    public void updateInputs(LEDIO.LEDIOInputs inputs) {
        inputs.currentState = currentState;
        inputs.totalLEDs = NUM_LEDS; // Use the configured number of LEDs
        inputs.vbat = candle.getBusVoltage();
        inputs.v5 = candle.get5VRailVoltage();
    }

    @Override
    public void setState(LEDState state) {
        currentState = state;
        // Wait a bit between configurations to prevent CAN bus flooding
        if (configWaitTimer.hasElapsed(CONFIG_WAIT_TIME)) {
            candle.setLEDs(state.r, state.g, state.b);
            configWaitTimer.reset();
            configWaitTimer.start();
        }
    }

    @Override
    public void setColor(int r, int g, int b) {
        if (configWaitTimer.hasElapsed(CONFIG_WAIT_TIME)) {
            candle.setLEDs(r, g, b);
            configWaitTimer.reset();
            configWaitTimer.start();
        }
    }

    @Override
    public void setLEDsSection(int startIndex, int count, int r, int g, int b) {
        if (configWaitTimer.hasElapsed(CONFIG_WAIT_TIME)) {
            candle.setLEDs(r, g, b, 0, startIndex, count);
            configWaitTimer.reset();
            configWaitTimer.start();
        }
    }

    @Override
    public void setAnimation(Animation animation) {
        if (configWaitTimer.hasElapsed(CONFIG_WAIT_TIME)) {
            candle.animate(animation);
            configWaitTimer.reset();
            configWaitTimer.start();
        }
    }

    @Override
    public void clearAnimation() {
        if (configWaitTimer.hasElapsed(CONFIG_WAIT_TIME)) {
            candle.clearAnimation(0);
            configWaitTimer.reset();
            configWaitTimer.start();
        }
    }
}

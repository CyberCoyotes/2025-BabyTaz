package frc.robot.subsystems.led;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDConfig {
    public static final class Constants {
        public static final int CANDLE_ID = 1;
        public static final int LED_COUNT = 300;
        public static final double DEFAULT_BRIGHTNESS = 0.9;
    }

    // Configuration properties
    public int ledCount;
    public double brightness;
    public LEDStripType stripType;
    public boolean statusLedOffWhenActive;
    public VBatOutputMode vBatOutputMode;
    public boolean disableWhenLOS;
    
    public static LEDConfig defaultConfig() {
        LEDConfig config = new LEDConfig();
        config.ledCount = Constants.LED_COUNT;
        config.brightness = Constants.DEFAULT_BRIGHTNESS;
        config.stripType = LEDStripType.GRB;
        config.statusLedOffWhenActive = true;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        config.disableWhenLOS = false;
        return config;
    }

    // Zone definition for LED segments
    public static class Zone {
        public final int startIndex;
        public final int length;
        public final String name;
        
        public Zone(int start, int length, String name) {
            this.startIndex = start;
            this.length = length;
            this.name = name;
        }
    }

    public List<Zone> zones = new ArrayList<>();
}


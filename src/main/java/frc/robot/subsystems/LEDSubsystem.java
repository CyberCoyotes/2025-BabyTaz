// LEDSubsystem.java
package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle;
    
    // Define some basic colors (RGB values)
    private static final int[] RED = {255, 0, 0};
    private static final int[] GREEN = {0, 255, 0};
    private static final int[] OFF = {0, 0, 0};

    public LEDSubsystem(int candleID) {
        candle = new CANdle(candleID);
        
        // Configure the CANdle
        CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = 0.90; // 90% brightness
        config.stripType = CANdle.LEDStripType.RGB; // Adjust if using different LED type
        candle.configAllSettings(config);
    }

    public void setColor(int[] rgb) {
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    public void setRed() {
        setColor(RED);
    }

    public void setGreen() {
        setColor(GREEN);
    }

    public void setOff() {
        setColor(OFF);
    }
}
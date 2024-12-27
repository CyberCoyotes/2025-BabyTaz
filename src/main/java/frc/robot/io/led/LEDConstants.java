package frc.robot.io.led;
// frc/robot/constants/LEDConstants.java

import com.ctre.phoenix.led.CANdle;

public final class LEDConstants {
    public static final int CANDLE_ID = 1;  // Set this to your CANdle's ID
    public static final int NUM_LEDS = 40;  // Set this to your LED strip length
    public static final CANdle.LEDStripType STRIP_TYPE = CANdle.LEDStripType.RGB;  // Set to your strip type
    
    private LEDConstants() {
        // Private constructor to prevent instantiation
    }
}
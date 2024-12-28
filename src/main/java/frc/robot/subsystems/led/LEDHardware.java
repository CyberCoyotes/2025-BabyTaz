package frc.robot.subsystems.led;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class LEDHardware {
    // Hardware state tracking
    private final CANdle candle;
    private Animation currentAnimation;
    private LEDConfig config;
    private boolean isConfigured = false;
    private int configRetryCount = 0;
    private static final int MAX_CONFIG_RETRIES = 3;
    private double lastUpdateTime = 0;
    
    // Diagnostic data structure (simplified from previous LEDIOInputs)
    public static class Status {
        public double busVoltage = 0.0;
        public double current = 0.0;
        public double temperature = 0.0;
        public boolean isConnected = false;
        public double lastUpdateTime = 0.0;
        public boolean isConfigured = false;
        public int configAttempts = 0;
    }
    
    public LEDHardware() {
        // Using the constant directly since we know we're using CANdle
        candle = new CANdle(LEDConfig.Constants.CANDLE_ID, "rio");
        config = LEDConfig.defaultConfig();
        DataLogManager.log("LEDHardware: Initialized with CANdle ID " + LEDConfig.Constants.CANDLE_ID);
    }

    public void configure(LEDConfig config) {
        this.config = config;
        configRetryCount = 0;
        attemptConfiguration();
    }

    private void attemptConfiguration() {
        try {
            CANdleConfiguration candleConfig = new CANdleConfiguration();
            candleConfig.stripType = config.stripType;
            candleConfig.brightnessScalar = config.brightness;
            candleConfig.statusLedOffWhenActive = config.statusLedOffWhenActive;
            candleConfig.vBatOutputMode = config.vBatOutputMode;
            candleConfig.disableWhenLOS = config.disableWhenLOS;

            var error = candle.configAllSettings(candleConfig, 100);
            if (error.value != 0) {
                handleConfigError(error);
                return;
            }

            isConfigured = true;
            configRetryCount = 0;
            DataLogManager.log("LEDHardware: Successfully configured");
        } catch (Exception e) {
            DataLogManager.log("LEDHardware: Configuration failed with exception: " + e.getMessage());
            handleConfigError(null);
        }
    }

    private void handleConfigError(ErrorCode error) {
        configRetryCount++;
        isConfigured = false;
        
        if (configRetryCount < MAX_CONFIG_RETRIES) {
            DataLogManager.log("LEDHardware: Configuration attempt " + configRetryCount + 
                             " failed. Error: " + (error != null ? error.toString() : "Unknown"));
            Timer.delay(0.1);
            attemptConfiguration();
        } else {
            DataLogManager.log("LEDHardware: Configuration failed after " + MAX_CONFIG_RETRIES + " attempts");
        }
    }

    public void setRGB(int r, int g, int b) {
        if (!isConfigured) {
            DataLogManager.log("LEDHardware: Attempted to set RGB before successful configuration");
            return;
        }

        try {
            currentAnimation = null;
            candle.setLEDs(r, g, b);
            lastUpdateTime = Timer.getFPGATimestamp();
        } catch (Exception e) {
            DataLogManager.log("LEDHardware: Failed to set RGB values: " + e.getMessage());
        }
    }

    public void setAnimation(Animation animation) {
        if (!isConfigured) {
            DataLogManager.log("LEDHardware: Attempted to set animation before successful configuration");
            return;
        }

        try {
            currentAnimation = animation;
            candle.animate(animation);
            lastUpdateTime = Timer.getFPGATimestamp();
        } catch (Exception e) {
            DataLogManager.log("LEDHardware: Failed to set animation: " + e.getMessage());
        }
    }

    public Status getStatus() {
        Status status = new Status();
        
        try {
            status.busVoltage = candle.getBusVoltage();
            status.current = candle.getCurrent();
            status.temperature = candle.getTemperature();
            status.isConnected = !candle.hasResetOccurred();
            status.lastUpdateTime = lastUpdateTime;
            status.isConfigured = isConfigured;
            status.configAttempts = configRetryCount;
        } catch (Exception e) {
            DataLogManager.log("LEDHardware: Failed to get status: " + e.getMessage());
            status.isConnected = false;
        }
        
        return status;
    }
}
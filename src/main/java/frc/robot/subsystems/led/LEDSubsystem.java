package frc.robot.subsystems.led;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;

public class LEDSubsystem extends SubsystemBase {
    private final LEDHardware hardware;
    private LEDState currentState = LEDState.OFF;
    private boolean animationEnabled = true;
    private double lastStateChangeTime = 0;

    // Animation configuration
    private double animationSpeed = 0.7;  // Default animation speed
    private double brightness = 1.0;      // Default brightness
    private final int ledCount;

    public LEDSubsystem() {
        this.hardware = new LEDHardware();
        this.ledCount = LEDConfig.Constants.LED_COUNT;
        
        DataLogManager.log("LEDSubsystem: Initializing...");
        hardware.configure(LEDConfig.defaultConfig());
        DataLogManager.log("LEDSubsystem: Initialization complete");
    }

    /**
     * Creates an appropriate animation based on the current state.
     * Each state can have its own unique animation pattern.
     */
    private Animation createStateAnimation() {
        // Convert brightness to the 0-255 range that CTRE animations expect
        int brightnessByte = (int)(brightness * 255);
        
        return switch(currentState) {
            // States that use rainbow animations
            case AUTONOMOUS -> new RainbowAnimation(
                brightnessByte,
                animationSpeed,
                ledCount
            );
            
            // States that use strobing animations
            case TARGET_VISIBLE, ERROR -> new StrobeAnimation(
                brightnessByte,
                currentState.r,
                currentState.g,
                currentState.b,
                animationSpeed,
                ledCount
            );
            
            // States that use "scanning" animations
            case INTAKING -> new LarsonAnimation(
                brightnessByte,
                currentState.r,
                currentState.g,
                currentState.b,
                animationSpeed,
                ledCount,
                LarsonAnimation.BounceMode.Front,
                7
            );
            
            // States that use color flow animations
            case SCORING -> new ColorFlowAnimation(
                brightnessByte,
                currentState.r,
                currentState.g,
                currentState.b,
                animationSpeed,
                ledCount,
                ColorFlowAnimation.Direction.Forward
            );
            
            // All other states don't use animations
            default -> null;
        };
    }

    @Override
    public void periodic() {
        var status = hardware.getStatus();
        
        if (status.isConfigured) {
            updateLEDs();
        } else if (status.configAttempts >= 3) {
            currentState = LEDState.ERROR;
            DataLogManager.log("LEDSubsystem: Hardware configuration failed, entering ERROR state");
        }
        
        updateTelemetry(status);
    }

    private void updateLEDs() {
        try {
            Animation stateAnimation = createStateAnimation();
            
            if (stateAnimation != null && animationEnabled) {
                hardware.setAnimation(stateAnimation);
            } else {
                hardware.setRGB(currentState.r, currentState.g, currentState.b);
            }
        } catch (Exception e) {
            DataLogManager.log("LEDSubsystem: Error updating LEDs: " + e.getMessage());
            currentState = LEDState.ERROR;
        }
    }

    // Public control methods
    public void setState(LEDState state) {
        if (currentState != state) {
            currentState = state;
            lastStateChangeTime = Timer.getFPGATimestamp();
            DataLogManager.log("LEDSubsystem: State changed to " + state.toString());
        }
    }

    public LEDState getState() {
        return currentState;
    }

    public void setAnimationEnabled(boolean enabled) {
        if (animationEnabled != enabled) {
            animationEnabled = enabled;
            updateLEDs();
            DataLogManager.log("LEDSubsystem: Animations " + (enabled ? "enabled" : "disabled"));
        }
    }

    /**
     * Adjust the speed of animations. Higher values make animations run faster.
     * @param speed Animation speed (typically 0.1 to 2.0)
     */
    public void setAnimationSpeed(double speed) {
        if (this.animationSpeed != speed) {
            this.animationSpeed = speed;
            updateLEDs();
        }
    }

    /**
     * Adjust the brightness of the LEDs.
     * @param brightness Brightness level (0.0 to 1.0)
     */
    public void setBrightness(double brightness) {
        if (this.brightness != brightness) {
            this.brightness = Math.min(1.0, Math.max(0.0, brightness));
            updateLEDs();
        }
    }

    private void updateTelemetry(LEDHardware.Status status) {
        SmartDashboard.putString("LED/State", currentState.toString());
        SmartDashboard.putBoolean("LED/AnimationsEnabled", animationEnabled);
        SmartDashboard.putNumber("LED/AnimationSpeed", animationSpeed);
        SmartDashboard.putNumber("LED/Brightness", brightness);
        SmartDashboard.putBoolean("LED/IsConfigured", status.isConfigured);
        SmartDashboard.putNumber("LED/BusVoltage", status.busVoltage);
        SmartDashboard.putNumber("LED/Current", status.current);
        SmartDashboard.putNumber("LED/Temperature", status.temperature);
        SmartDashboard.putBoolean("LED/IsConnected", status.isConnected);
    }
}
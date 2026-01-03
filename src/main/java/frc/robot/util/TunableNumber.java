// TunableNumber.java
package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TunableNumber - Utility for live-tunable parameters via SmartDashboard/Shuffleboard.
 *
 * Allows real-time adjustment of PID gains, tolerances, and other constants
 * without redeploying code.
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private double lastValue;
    private static boolean tuningEnabled = true;

    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = dashboardKey;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        if (tuningEnabled) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    public double get() {
        if (!tuningEnabled) {
            return defaultValue;
        }
        lastValue = SmartDashboard.getNumber(key, defaultValue);
        return lastValue;
    }

    public void set(double value) {
        if (tuningEnabled) {
            SmartDashboard.putNumber(key, value);
            lastValue = value;
        }
    }

    public void reset() {
        set(defaultValue);
    }

    public boolean hasChanged() {
        return lastValue != SmartDashboard.getNumber(key, defaultValue);
    }

    public double getDefault() {
        return defaultValue;
    }

    public String getKey() {
        return key;
    }

    /**
     * Disable tuning for competitions (all TunableNumbers return default values)
     */
    public static void setTuningEnabled(boolean enabled) {
        tuningEnabled = enabled;
    }

    public static boolean isTuningEnabled() {
        return tuningEnabled;
    }
}
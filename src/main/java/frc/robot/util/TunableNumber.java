// TunableNumber.java
package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

@SuppressWarnings("unused")


/**
 * TunableNumber - Utility for live-tunable parameters via SmartDashboard/Shuffleboard.
 *
 * Allows real-time adjustment of PID gains, tolerances, and other constants
 * without redeploying code.
 *
 * Features:
 * - Supports both SmartDashboard and Shuffleboard
 * - Organized display with custom positioning and sizing
 * - Change detection for efficient PID updates
 * - Competition mode to lock values
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private double lastValue;
    private static boolean tuningEnabled = true;
    private GenericEntry shuffleboardEntry = null;

    /**
     * Creates a TunableNumber that appears in SmartDashboard.
     *
     * @param dashboardKey Key for SmartDashboard/Shuffleboard
     * @param defaultValue Default value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = dashboardKey;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        if (tuningEnabled) {
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    /**
     * Creates a TunableNumber with Shuffleboard integration.
     *
     * @param tab Shuffleboard tab to add this number to
     * @param title Display title
     * @param defaultValue Default value
     * @param col Column position (0-indexed)
     * @param row Row position (0-indexed)
     * @param width Widget width
     * @param height Widget height
     */
    public TunableNumber(ShuffleboardTab tab, String title, double defaultValue,
                         int col, int row, int width, int height) {
        this.key = tab.getTitle() + "/" + title;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        if (tuningEnabled) {
            SimpleWidget widget = tab.add(title, defaultValue)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withPosition(col, row)
                .withSize(width, height);

            // Add properties for better UX
            if (title.toLowerCase().contains("tolerance") || title.toLowerCase().contains("threshold")) {
                widget.withProperties(Map.of("min", 0.0, "max", 10.0));
            } else if (title.toLowerCase().contains("speed")) {
                widget.withProperties(Map.of("min", 0.0, "max", 5.0));
            } else if (title.toLowerCase().contains("kp") || title.toLowerCase().contains("ki") ||
                       title.toLowerCase().contains("kd")) {
                widget.withProperties(Map.of("min", 0.0, "max", 2.0));
            }

            shuffleboardEntry = widget.getEntry();
            SmartDashboard.putNumber(key, defaultValue);
        }
    }

    public double get() {
        if (!tuningEnabled) {
            return defaultValue;
        }

        if (shuffleboardEntry != null) {
            lastValue = shuffleboardEntry.getDouble(defaultValue);
        } else {
            lastValue = SmartDashboard.getNumber(key, defaultValue);
        }
        return lastValue;
    }

    public void set(double value) {
        if (tuningEnabled) {
            if (shuffleboardEntry != null) {
                shuffleboardEntry.setDouble(value);
            }
            SmartDashboard.putNumber(key, value);
            lastValue = value;
        }
    }

    public void reset() {
        set(defaultValue);
    }

    public boolean hasChanged() {
        double currentValue;
        if (shuffleboardEntry != null) {
            currentValue = shuffleboardEntry.getDouble(defaultValue);
        } else {
            currentValue = SmartDashboard.getNumber(key, defaultValue);
        }
        return Math.abs(lastValue - currentValue) > 1e-9; // Use epsilon for double comparison
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
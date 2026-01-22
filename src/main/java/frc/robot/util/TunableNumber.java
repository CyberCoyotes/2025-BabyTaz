// TunableNumber.java
package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

@SuppressWarnings("unused")


/**
 * TunableNumber - Utility for live-tunable parameters via Elastic Dashboard.
 *
 * Allows real-time adjustment of PID gains, tolerances, and other constants
 * without redeploying code.
 *
 * Features:
 * - Uses NetworkTables for Elastic Dashboard integration
 * - Change detection for efficient PID updates
 * - Competition mode to lock values
 * - Organized display in Elastic tabs
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private double lastValue;
    private static boolean tuningEnabled = true;
    private DoubleEntry ntEntry = null;

    /**
     * Creates a TunableNumber that appears in Elastic Dashboard.
     *
     * @param dashboardKey Key for NetworkTables/Elastic
     * @param defaultValue Default value
     */
    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = dashboardKey;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        if (tuningEnabled) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic");
            DoubleTopic topic = table.getDoubleTopic(key);
            ntEntry = topic.getEntry(defaultValue);
            ntEntry.set(defaultValue);
        }
    }

    /**
     * Creates a TunableNumber with Elastic Dashboard integration.
     * Organizes values in tabs for better dashboard layout.
     *
     * @param tabName Tab name for organization in Elastic
     * @param title Display title
     * @param defaultValue Default value
     * @param col Column position (ignored, for backward compatibility)
     * @param row Row position (ignored, for backward compatibility)
     * @param width Widget width (ignored, for backward compatibility)
     * @param height Widget height (ignored, for backward compatibility)
     */
    public TunableNumber(String tabName, String title, double defaultValue,
                         int col, int row, int width, int height) {
        this.key = tabName + "/" + title;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        if (tuningEnabled) {
            NetworkTable table = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable(tabName);
            DoubleTopic topic = table.getDoubleTopic(title);
            ntEntry = topic.getEntry(defaultValue);
            ntEntry.set(defaultValue);
        }
    }

    public double get() {
        if (!tuningEnabled) {
            return defaultValue;
        }

        if (ntEntry != null) {
            lastValue = ntEntry.get(defaultValue);
        }
        return lastValue;
    }

    public void set(double value) {
        if (tuningEnabled && ntEntry != null) {
            ntEntry.set(value);
            lastValue = value;
        }
    }

    public void reset() {
        set(defaultValue);
    }

    public boolean hasChanged() {
        if (ntEntry == null) {
            return false;
        }
        double currentValue = ntEntry.get(defaultValue);
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
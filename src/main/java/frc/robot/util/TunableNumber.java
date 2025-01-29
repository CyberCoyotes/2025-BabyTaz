// TunableNumber.java
package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private double lastValue;

    public TunableNumber(String dashboardKey, double defaultValue) {
        this.key = dashboardKey;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        SmartDashboard.putNumber(key, defaultValue);
    }

    public double get() {
        lastValue = SmartDashboard.getNumber(key, defaultValue);
        return lastValue;
    }

    public boolean hasChanged() {
        return lastValue != SmartDashboard.getNumber(key, defaultValue);
    }
}
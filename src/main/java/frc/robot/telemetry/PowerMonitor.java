package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Monitors robot power systems including battery voltage, current draw, and brownout status.
 * Provides warnings when voltage drops below safe thresholds.
 */
public class PowerMonitor {
    private final NetworkTable powerTable;
    private double minVoltage;
    private boolean lowVoltageWarned;

    private static final double LOW_VOLTAGE_THRESHOLD = 11.5;

    /**
     * Creates a new PowerMonitor instance.
     * Initializes NetworkTables connection to Elastic/Power path.
     */
    public PowerMonitor() {
        this.powerTable = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Power");
        this.minVoltage = Double.MAX_VALUE;
        this.lowVoltageWarned = false;
    }

    /**
     * Updates and publishes all power-related data to NetworkTables.
     * Monitors battery voltage, current draw, and brownout status.
     * Sends warnings to DriverStation if voltage drops below safe threshold.
     */
    public void update() {
        // Get current power data from RobotController
        double voltage = RobotController.getBatteryVoltage();
        double current = RobotController.getInputCurrent();
        boolean brownedOut = RobotController.isBrownedOut();

        // Track minimum voltage
        if (voltage < minVoltage) {
            minVoltage = voltage;
            powerTable.getEntry("MinVoltage").setDouble(minVoltage);
        }

        // Publish current power data
        powerTable.getEntry("BatteryVoltage").setDouble(voltage);
        powerTable.getEntry("InputCurrent").setDouble(current);
        powerTable.getEntry("BrownedOut").setBoolean(brownedOut);

        // Calculate power consumption (watts)
        double power = voltage * current;
        powerTable.getEntry("PowerWatts").setDouble(power);

        // Check for low voltage condition
        if (voltage < LOW_VOLTAGE_THRESHOLD && !lowVoltageWarned) {
            String warningMessage = String.format(
                "LOW BATTERY VOLTAGE: %.2fV (threshold: %.1fV)",
                voltage,
                LOW_VOLTAGE_THRESHOLD
            );
            DriverStation.reportWarning(warningMessage, false);
            lowVoltageWarned = true;
        }

        // Reset warning flag if voltage recovers
        if (voltage >= LOW_VOLTAGE_THRESHOLD + 0.5) {
            lowVoltageWarned = false;
        }

        // Warn about brownout condition
        if (brownedOut) {
            DriverStation.reportWarning("BROWNOUT DETECTED - Check battery and connections!", false);
        }
    }

    /**
     * Resets the minimum voltage tracker.
     * Useful when starting a new match or installing a fresh battery.
     */
    public void resetMinVoltage() {
        minVoltage = Double.MAX_VALUE;
        lowVoltageWarned = false;
    }

    /**
     * Gets the minimum voltage recorded since last reset.
     *
     * @return Minimum battery voltage in volts
     */
    public double getMinVoltage() {
        return (minVoltage == Double.MAX_VALUE) ? 0.0 : minVoltage;
    }
}

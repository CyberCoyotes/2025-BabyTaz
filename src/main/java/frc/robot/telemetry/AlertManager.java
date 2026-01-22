package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Manages actionable alerts for drivers and pit crew.
 * Sends alerts to both Elastic Dashboard and DriverStation console.
 */
public class AlertManager {
    private final NetworkTable alertTable;
    private int alertCounter;

    /**
     * Alert severity levels.
     */
    public enum AlertLevel {
        /** Informational message - no action required */
        INFO,
        /** Warning - attention recommended */
        WARNING,
        /** Error - immediate action required */
        ERROR
    }

    /**
     * Creates a new AlertManager instance.
     * Initializes NetworkTables connection to Elastic/Alerts path.
     */
    public AlertManager() {
        this.alertTable = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Alerts");
        this.alertCounter = 0;
    }

    /**
     * Sends an alert to both Elastic Dashboard and DriverStation.
     * Alerts are timestamped and categorized by severity level.
     *
     * @param message The alert message to send
     * @param level The severity level of the alert
     */
    public void sendAlert(String message, AlertLevel level) {
        // Increment alert counter for unique identification
        alertCounter++;

        // Create timestamp
        double timestamp = Timer.getFPGATimestamp();

        // Create alert entry with counter for uniqueness
        String alertKey = String.format("Alert_%d", alertCounter);
        NetworkTable specificAlert = alertTable.getSubTable(alertKey);

        // Publish alert data to NetworkTables
        specificAlert.getEntry("Message").setString(message);
        specificAlert.getEntry("Level").setString(level.toString());
        specificAlert.getEntry("Timestamp").setDouble(timestamp);

        // Also publish to a "Latest" alert for easy dashboard access
        alertTable.getEntry("LatestMessage").setString(message);
        alertTable.getEntry("LatestLevel").setString(level.toString());
        alertTable.getEntry("LatestTimestamp").setDouble(timestamp);
        alertTable.getEntry("AlertCount").setInteger(alertCounter);

        // Send to DriverStation based on level
        switch (level) {
            case ERROR:
                DriverStation.reportError("[ALERT] " + message, false);
                break;
            case WARNING:
                DriverStation.reportWarning("[ALERT] " + message, false);
                break;
            case INFO:
                // Info messages go to standard output
                System.out.println("[INFO ALERT] " + message);
                break;
        }
    }

    /**
     * Sends an informational alert.
     * Convenience method for INFO level alerts.
     *
     * @param message The alert message to send
     */
    public void sendInfo(String message) {
        sendAlert(message, AlertLevel.INFO);
    }

    /**
     * Sends a warning alert.
     * Convenience method for WARNING level alerts.
     *
     * @param message The alert message to send
     */
    public void sendWarning(String message) {
        sendAlert(message, AlertLevel.WARNING);
    }

    /**
     * Sends an error alert.
     * Convenience method for ERROR level alerts.
     *
     * @param message The alert message to send
     */
    public void sendError(String message) {
        sendAlert(message, AlertLevel.ERROR);
    }

    /**
     * Clears all alerts from the dashboard.
     * Resets the alert counter.
     */
    public void clearAlerts() {
        alertCounter = 0;
        alertTable.getEntry("AlertCount").setInteger(0);
        alertTable.getEntry("LatestMessage").setString("");
        alertTable.getEntry("LatestLevel").setString("");
    }

    /**
     * Gets the total number of alerts sent since initialization or last clear.
     *
     * @return Total alert count
     */
    public int getAlertCount() {
        return alertCounter;
    }
}

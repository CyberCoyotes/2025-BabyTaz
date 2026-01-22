package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

/**
 * Monitors robot performance metrics including cycle times.
 * Tracks loop execution time to identify performance bottlenecks.
 */
public class PerformanceMonitor {
    private final NetworkTable performanceTable;
    private double lastCycleTime;
    private double maxCycleTime;

    /**
     * Creates a new PerformanceMonitor instance.
     * Initializes NetworkTables connection to Elastic/Performance path.
     */
    public PerformanceMonitor() {
        this.performanceTable = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Performance");
        this.lastCycleTime = Timer.getFPGATimestamp();
        this.maxCycleTime = 0.0;
    }

    /**
     * Updates and tracks robot cycle time.
     * Call this method at the end of each robot periodic loop to measure execution time.
     * Automatically tracks and publishes maximum cycle time encountered.
     */
    public void updateCycleTime() {
        double currentTime = Timer.getFPGATimestamp();
        double cycleTime = (currentTime - lastCycleTime) * 1000.0; // Convert to milliseconds

        // Update maximum cycle time if current exceeds it
        if (cycleTime > maxCycleTime) {
            maxCycleTime = cycleTime;
            performanceTable.getEntry("MaxCycleTimeMs").setDouble(maxCycleTime);
        }

        // Publish current cycle time
        performanceTable.getEntry("CycleTimeMs").setDouble(cycleTime);

        // Store current time for next cycle calculation
        lastCycleTime = currentTime;
    }

    /**
     * Resets the maximum cycle time counter.
     * Useful when starting a new match or testing session.
     */
    public void resetMaxCycleTime() {
        maxCycleTime = 0.0;
        performanceTable.getEntry("MaxCycleTimeMs").setDouble(0.0);
    }

    /**
     * Gets the current maximum cycle time in milliseconds.
     *
     * @return Maximum cycle time recorded since last reset
     */
    public double getMaxCycleTime() {
        return maxCycleTime;
    }
}

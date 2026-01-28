package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Hopper subsystem that monitors fill level using three CANrange ToF sensors.
 *
 * Uses time-of-flight distance measurements to determine hopper fill percentage.
 * As the hopper fills with balls, the measured distance decreases.
 *
 * Sensor placement: Three CANrange sensors (A, B, C) mounted at top of hopper,
 * pointing down toward the hopper floor.
 */
public class HopperSubsystem extends SubsystemBase {

    // ====== CAN IDs (TODO: Update with actual values) ======
    public static final int HOPPER_TOF_A_ID = 31;
    public static final int HOPPER_TOF_B_ID = 32;
    public static final int HOPPER_TOF_C_ID = 33;

    // ====== CONSTANTS ======
    /** Maximum distance from sensor to hopper floor when empty (meters) */
    private static final double HOPPER_MAX_DISTANCE_METERS = 0.50; // 50 cm = 0.50 m

    /** Minimum distance when hopper is completely full (meters) */
    private static final double HOPPER_MIN_DISTANCE_METERS = 0.0; // 0 cm = 0.0 m

    /** Minimum time (seconds) that readings must be stable before updating status */
    private static final double STABILITY_TIME_SECONDS = 0.25; // 250 ms debounce

    /** Threshold for "full" status (percentage, 0.0-1.0) */
    private static final double FULL_THRESHOLD = 0.90; // 90% full

    /** Threshold for "empty" status (percentage, 0.0-1.0) */
    private static final double EMPTY_THRESHOLD = 0.10; // 10% full

    /** Maximum allowed deviation between readings before considering them unstable (meters) */
    private static final double STABILITY_TOLERANCE_METERS = 0.05; // 5 cm

    // ====== HARDWARE ======
    private final CANrange hopperAToF = new CANrange(HOPPER_TOF_A_ID);
    private final CANrange hopperBToF = new CANrange(HOPPER_TOF_B_ID);
    private final CANrange hopperCToF = new CANrange(HOPPER_TOF_C_ID);

    // ====== STATUS SIGNALS ======
    private final StatusSignal<Distance> distanceA;
    private final StatusSignal<Distance> distanceB;
    private final StatusSignal<Distance> distanceC;

    // ====== STATE ======
    private double hopperAverageDistance = HOPPER_MAX_DISTANCE_METERS; // Current average distance (meters)
    private double hopperStatus = 0.0; // Hopper fill percentage (0.0 to 1.0)
    private boolean isHopperFull = false;
    private boolean isHopperEmpty = true;

    // Stability tracking
    private double lastStableDistance = HOPPER_MAX_DISTANCE_METERS;
    private double stableStartTime = 0.0;
    private boolean isReadingStable = false;

    // ====== TELEMETRY ======
    private final NetworkTable elasticTable;

    public HopperSubsystem() {
        // Cache status signals for distance readings
        // CANrange getDistance() returns Distance in meters (Phoenix 6 native unit)
        distanceA = hopperAToF.getDistance();
        distanceB = hopperBToF.getDistance();
        distanceC = hopperCToF.getDistance();

        // Set update frequencies (10 Hz is sufficient for hopper level monitoring)
        BaseStatusSignal.setUpdateFrequencyForAll(10, distanceA, distanceB, distanceC);

        // Optimize CAN bus usage
        hopperAToF.optimizeBusUtilization();
        hopperBToF.optimizeBusUtilization();
        hopperCToF.optimizeBusUtilization();

        // Elastic Dashboard table
        elasticTable = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("Hopper");
    }

    @Override
    public void periodic() {
        // Refresh all distance signals in one batch
        BaseStatusSignal.refreshAll(distanceA, distanceB, distanceC);

        // Get individual distances (in meters)
        double distA = distanceA.getValueAsDouble();
        double distB = distanceB.getValueAsDouble();
        double distC = distanceC.getValueAsDouble();

        // Calculate average distance
        double currentAverage = (distA + distB + distC) / 3.0;

        // Check reading stability (to filter out ball jostling)
        updateStability(currentAverage);

        // Only update hopper status when readings are stable
        if (isReadingStable) {
            hopperAverageDistance = currentAverage;

            // Calculate fill percentage
            // When distance = MAX, hopper is empty (0%)
            // When distance = MIN, hopper is full (100%)
            double clampedDistance = Math.max(HOPPER_MIN_DISTANCE_METERS,
                    Math.min(HOPPER_MAX_DISTANCE_METERS, hopperAverageDistance));

            // Invert: shorter distance = more full
            hopperStatus = 1.0 - (clampedDistance / HOPPER_MAX_DISTANCE_METERS);

            // Update boolean flags
            isHopperFull = hopperStatus >= FULL_THRESHOLD;
            isHopperEmpty = hopperStatus <= EMPTY_THRESHOLD;
        }

        updateTelemetry(distA, distB, distC);
    }

    /**
     * Tracks reading stability over time to filter out transient changes
     * caused by balls bouncing or jostling.
     */
    private void updateStability(double currentAverage) {
        double deviation = Math.abs(currentAverage - lastStableDistance);
        double currentTime = Timer.getFPGATimestamp();

        if (deviation > STABILITY_TOLERANCE_METERS) {
            // Reading changed significantly, reset stability timer
            lastStableDistance = currentAverage;
            stableStartTime = currentTime;
            isReadingStable = false;
        } else {
            // Reading is within tolerance, check if stable long enough
            double stableDuration = currentTime - stableStartTime;
            isReadingStable = stableDuration >= STABILITY_TIME_SECONDS;
        }
    }

    // ====== PUBLIC API ======

    /**
     * Get the hopper fill level as a percentage (0.0 to 1.0).
     * @return Fill percentage where 0.0 = empty, 1.0 = full
     */
    public double getHopperStatus() {
        return hopperStatus;
    }

    /**
     * Get the hopper fill level as a percentage (0 to 100).
     * @return Fill percentage where 0 = empty, 100 = full
     */
    public double getHopperStatusPercent() {
        return hopperStatus * 100.0;
    }

    /**
     * Check if the hopper is full (above threshold).
     * @return true if hopper is at or above 90% capacity
     */
    public boolean isHopperFull() {
        return isHopperFull;
    }

    /**
     * Check if the hopper is empty (below threshold).
     * @return true if hopper is at or below 10% capacity
     */
    public boolean isHopperEmpty() {
        return isHopperEmpty;
    }

    /**
     * Get the average distance from sensors to hopper contents (meters).
     * @return Average distance in meters
     */
    public double getAverageDistanceMeters() {
        return hopperAverageDistance;
    }

    /**
     * Get the average distance from sensors to hopper contents (centimeters).
     * @return Average distance in centimeters
     */
    public double getAverageDistanceCm() {
        return hopperAverageDistance * 100.0;
    }

    /**
     * Check if the current readings are considered stable.
     * @return true if readings have been stable for the required duration
     */
    public boolean isReadingStable() {
        return isReadingStable;
    }

    /**
     * Get individual sensor distance (meters).
     * @param sensorIndex 0=A, 1=B, 2=C
     * @return Distance in meters, or -1 if invalid index
     */
    public double getSensorDistanceMeters(int sensorIndex) {
        return switch (sensorIndex) {
            case 0 -> distanceA.getValueAsDouble();
            case 1 -> distanceB.getValueAsDouble();
            case 2 -> distanceC.getValueAsDouble();
            default -> -1.0;
        };
    }

    // ====== TELEMETRY ======

    private void updateTelemetry(double distA, double distB, double distC) {
        // AdvantageKit logging
        Logger.recordOutput("Hopper/Status", hopperStatus);
        Logger.recordOutput("Hopper/StatusPercent", hopperStatus * 100.0);
        Logger.recordOutput("Hopper/IsFull", isHopperFull);
        Logger.recordOutput("Hopper/IsEmpty", isHopperEmpty);
        Logger.recordOutput("Hopper/AverageDistanceM", hopperAverageDistance);
        Logger.recordOutput("Hopper/AverageDistanceCm", hopperAverageDistance * 100.0);
        Logger.recordOutput("Hopper/IsStable", isReadingStable);
        Logger.recordOutput("Hopper/SensorA_DistanceM", distA);
        Logger.recordOutput("Hopper/SensorB_DistanceM", distB);
        Logger.recordOutput("Hopper/SensorC_DistanceM", distC);

        // Elastic Dashboard (live display)
        elasticTable.getEntry("StatusPercent").setDouble(hopperStatus * 100.0);
        elasticTable.getEntry("IsFull").setBoolean(isHopperFull);
        elasticTable.getEntry("IsEmpty").setBoolean(isHopperEmpty);
        elasticTable.getEntry("AverageDistanceCm").setDouble(hopperAverageDistance * 100.0);
        elasticTable.getEntry("IsStable").setBoolean(isReadingStable);
        elasticTable.getEntry("SensorA_Cm").setDouble(distA * 100.0);
        elasticTable.getEntry("SensorB_Cm").setDouble(distB * 100.0);
        elasticTable.getEntry("SensorC_Cm").setDouble(distC * 100.0);
    }
}

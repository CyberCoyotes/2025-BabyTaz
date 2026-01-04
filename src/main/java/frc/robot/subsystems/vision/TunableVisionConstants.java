package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import java.util.Map;

/**
 * Tunable vision constants that automatically create Shuffleboard tabs with sliders.
 *
 * This class programmatically creates Shuffleboard tabs for each vision model with
 * pre-configured sliders. No JSON file or manual drag-and-drop needed!
 *
 * USAGE:
 * 1. Call TunableVisionConstants.initializeAll() during robot initialization
 * 2. Access constants via TunableVisionConstants.ModelA.getRotationKp()
 * 3. Use .hasChanged() to detect updates and refresh PID controllers
 *
 * Shuffleboard tabs are automatically created:
 * - "Vision A: Rotation" - Model A constants
 * - "Vision B: Rot+Range" - Model B constants
 * - "Vision C: Full 3-Axis" - Model C constants
 * - "Vision D: Color Hunt" - Model D constants
 */
public class TunableVisionConstants {

    private static boolean initialized = false;

    // ============================================================================
    // MODEL A: ROTATION ONLY
    // ============================================================================
    public static class ModelA {
        private static GenericEntry rotation_kP;
        private static GenericEntry rotation_kI;
        private static GenericEntry rotation_kD;
        private static GenericEntry maxRotationSpeed;
        private static GenericEntry minRotationSpeed;
        private static GenericEntry rotationTolerance_deg;

        private static double lastKp, lastKi, lastKd, lastTolerance;

        public static void initialize() {
            ShuffleboardTab tab = Shuffleboard.getTab("Vision A: Rotation");

            // PID Gains
            rotation_kP = tab.add("kP", VisionConstants.ModelA.ROTATION_KP)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.2, "block_increment", 0.001))
                .withPosition(0, 0).withSize(2, 1).getEntry();

            rotation_kI = tab.add("kI", VisionConstants.ModelA.ROTATION_KI)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.05, "block_increment", 0.001))
                .withPosition(0, 1).withSize(2, 1).getEntry();

            rotation_kD = tab.add("kD", VisionConstants.ModelA.ROTATION_KD)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.05, "block_increment", 0.001))
                .withPosition(0, 2).withSize(2, 1).getEntry();

            // Speed limits
            maxRotationSpeed = tab.add("Max Speed (rad/s)", VisionConstants.ModelA.MAX_ROTATION_SPEED_RADPS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 4.0, "block_increment", 0.1))
                .withPosition(2, 0).withSize(2, 1).getEntry();

            minRotationSpeed = tab.add("Min Speed (rad/s)", VisionConstants.ModelA.MIN_ROTATION_SPEED_RADPS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.5, "block_increment", 0.01))
                .withPosition(2, 1).withSize(2, 1).getEntry();

            // Tolerance
            rotationTolerance_deg = tab.add("Tolerance (deg)", VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.5, "max", 5.0, "block_increment", 0.1))
                .withPosition(2, 2).withSize(2, 1).getEntry();

            // Initialize last values
            lastKp = VisionConstants.ModelA.ROTATION_KP;
            lastKi = VisionConstants.ModelA.ROTATION_KI;
            lastKd = VisionConstants.ModelA.ROTATION_KD;
            lastTolerance = VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES;
        }

        // Getters
        public static double getRotationKp() { return rotation_kP.getDouble(VisionConstants.ModelA.ROTATION_KP); }
        public static double getRotationKi() { return rotation_kI.getDouble(VisionConstants.ModelA.ROTATION_KI); }
        public static double getRotationKd() { return rotation_kD.getDouble(VisionConstants.ModelA.ROTATION_KD); }
        public static double getMaxRotationSpeed() { return maxRotationSpeed.getDouble(VisionConstants.ModelA.MAX_ROTATION_SPEED_RADPS); }
        public static double getMinRotationSpeed() { return minRotationSpeed.getDouble(VisionConstants.ModelA.MIN_ROTATION_SPEED_RADPS); }
        public static double getRotationTolerance() { return rotationTolerance_deg.getDouble(VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES); }

        // hasChanged methods for PID auto-update
        public static boolean rotationKpHasChanged() {
            double current = getRotationKp();
            if (current != lastKp) {
                lastKp = current;
                return true;
            }
            return false;
        }

        public static boolean rotationKiHasChanged() {
            double current = getRotationKi();
            if (current != lastKi) {
                lastKi = current;
                return true;
            }
            return false;
        }

        public static boolean rotationKdHasChanged() {
            double current = getRotationKd();
            if (current != lastKd) {
                lastKd = current;
                return true;
            }
            return false;
        }

        public static boolean rotationToleranceHasChanged() {
            double current = getRotationTolerance();
            if (current != lastTolerance) {
                lastTolerance = current;
                return true;
            }
            return false;
        }

        public static boolean anyPIDHasChanged() {
            return rotationKpHasChanged() || rotationKiHasChanged() || rotationKdHasChanged();
        }
    }

    // ============================================================================
    // MODEL B: ROTATION + RANGE
    // ============================================================================
    public static class ModelB {
        // Rotation PID
        private static GenericEntry rotation_kP;
        private static GenericEntry rotation_kI;
        private static GenericEntry rotation_kD;

        // Range PID
        private static GenericEntry range_kP;
        private static GenericEntry range_kI;
        private static GenericEntry range_kD;

        // Speed limits
        private static GenericEntry maxRotationSpeed;
        private static GenericEntry maxForwardSpeed;

        // Tolerances
        private static GenericEntry rotationTolerance_deg;
        private static GenericEntry distanceTolerance_m;
        private static GenericEntry targetDistance_m;

        private static double lastRotKp, lastRotKi, lastRotKd;
        private static double lastRangeKp, lastRangeKi, lastRangeKd;
        private static double lastRotTol, lastDistTol, lastTargetDist;

        public static void initialize() {
            ShuffleboardTab tab = Shuffleboard.getTab("Vision B: Rot+Range");

            // Rotation PID (left column)
            rotation_kP = tab.add("Rot kP", VisionConstants.ModelB.ROTATION_KP)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.15, "block_increment", 0.001))
                .withPosition(0, 0).withSize(2, 1).getEntry();
            rotation_kI = tab.add("Rot kI", VisionConstants.ModelB.ROTATION_KI)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.05, "block_increment", 0.001))
                .withPosition(0, 1).withSize(2, 1).getEntry();
            rotation_kD = tab.add("Rot kD", VisionConstants.ModelB.ROTATION_KD)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.05, "block_increment", 0.001))
                .withPosition(0, 2).withSize(2, 1).getEntry();

            // Range PID (middle column)
            range_kP = tab.add("Range kP", VisionConstants.ModelB.RANGE_KP)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 3.0, "block_increment", 0.05))
                .withPosition(2, 0).withSize(2, 1).getEntry();
            range_kI = tab.add("Range kI", VisionConstants.ModelB.RANGE_KI)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.5, "block_increment", 0.01))
                .withPosition(2, 1).withSize(2, 1).getEntry();
            range_kD = tab.add("Range kD", VisionConstants.ModelB.RANGE_KD)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 0.5, "block_increment", 0.01))
                .withPosition(2, 2).withSize(2, 1).getEntry();

            // Speed limits (right column)
            maxRotationSpeed = tab.add("Max Rot (rad/s)", VisionConstants.ModelB.MAX_ROTATION_SPEED_RADPS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 4.0, "block_increment", 0.1))
                .withPosition(4, 0).withSize(2, 1).getEntry();
            maxForwardSpeed = tab.add("Max Fwd (m/s)", VisionConstants.ModelB.MAX_FORWARD_SPEED_MPS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0, "max", 2.0, "block_increment", 0.05))
                .withPosition(4, 1).withSize(2, 1).getEntry();

            // Tolerances (bottom row)
            rotationTolerance_deg = tab.add("Rot Tol (deg)", VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.5, "max", 5.0, "block_increment", 0.1))
                .withPosition(0, 3).withSize(2, 1).getEntry();
            distanceTolerance_m = tab.add("Dist Tol (m)", VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.01, "max", 0.3, "block_increment", 0.01))
                .withPosition(2, 3).withSize(2, 1).getEntry();
            targetDistance_m = tab.add("Target Dist (m)", VisionConstants.DEFAULT_TARGET_DISTANCE_METERS)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.3, "max", 2.0, "block_increment", 0.05))
                .withPosition(4, 3).withSize(2, 1).getEntry();

            // Initialize last values
            lastRotKp = VisionConstants.ModelB.ROTATION_KP;
            lastRotKi = VisionConstants.ModelB.ROTATION_KI;
            lastRotKd = VisionConstants.ModelB.ROTATION_KD;
            lastRangeKp = VisionConstants.ModelB.RANGE_KP;
            lastRangeKi = VisionConstants.ModelB.RANGE_KI;
            lastRangeKd = VisionConstants.ModelB.RANGE_KD;
            lastRotTol = VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES;
            lastDistTol = VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS;
            lastTargetDist = VisionConstants.DEFAULT_TARGET_DISTANCE_METERS;
        }

        // Rotation getters
        public static double getRotationKp() { return rotation_kP.getDouble(VisionConstants.ModelB.ROTATION_KP); }
        public static double getRotationKi() { return rotation_kI.getDouble(VisionConstants.ModelB.ROTATION_KI); }
        public static double getRotationKd() { return rotation_kD.getDouble(VisionConstants.ModelB.ROTATION_KD); }

        // Range getters
        public static double getRangeKp() { return range_kP.getDouble(VisionConstants.ModelB.RANGE_KP); }
        public static double getRangeKi() { return range_kI.getDouble(VisionConstants.ModelB.RANGE_KI); }
        public static double getRangeKd() { return range_kD.getDouble(VisionConstants.ModelB.RANGE_KD); }

        // Speed limits getters
        public static double getMaxRotationSpeed() { return maxRotationSpeed.getDouble(VisionConstants.ModelB.MAX_ROTATION_SPEED_RADPS); }
        public static double getMaxForwardSpeed() { return maxForwardSpeed.getDouble(VisionConstants.ModelB.MAX_FORWARD_SPEED_MPS); }

        // Tolerance getters
        public static double getRotationTolerance() { return rotationTolerance_deg.getDouble(VisionConstants.ModelB.ROTATION_TOLERANCE_DEGREES); }
        public static double getDistanceTolerance() { return distanceTolerance_m.getDouble(VisionConstants.ModelB.DISTANCE_TOLERANCE_METERS); }
        public static double getTargetDistance() { return targetDistance_m.getDouble(VisionConstants.DEFAULT_TARGET_DISTANCE_METERS); }

        // hasChanged methods
        public static boolean rotationKpHasChanged() {
            double current = getRotationKp();
            if (current != lastRotKp) { lastRotKp = current; return true; }
            return false;
        }
        public static boolean rotationKiHasChanged() {
            double current = getRotationKi();
            if (current != lastRotKi) { lastRotKi = current; return true; }
            return false;
        }
        public static boolean rotationKdHasChanged() {
            double current = getRotationKd();
            if (current != lastRotKd) { lastRotKd = current; return true; }
            return false;
        }
        public static boolean rangeKpHasChanged() {
            double current = getRangeKp();
            if (current != lastRangeKp) { lastRangeKp = current; return true; }
            return false;
        }
        public static boolean rangeKiHasChanged() {
            double current = getRangeKi();
            if (current != lastRangeKi) { lastRangeKi = current; return true; }
            return false;
        }
        public static boolean rangeKdHasChanged() {
            double current = getRangeKd();
            if (current != lastRangeKd) { lastRangeKd = current; return true; }
            return false;
        }
        public static boolean rotationToleranceHasChanged() {
            double current = getRotationTolerance();
            if (current != lastRotTol) { lastRotTol = current; return true; }
            return false;
        }
        public static boolean distanceToleranceHasChanged() {
            double current = getDistanceTolerance();
            if (current != lastDistTol) { lastDistTol = current; return true; }
            return false;
        }
        public static boolean targetDistanceHasChanged() {
            double current = getTargetDistance();
            if (current != lastTargetDist) { lastTargetDist = current; return true; }
            return false;
        }

        public static boolean rotationPIDHasChanged() {
            return rotationKpHasChanged() || rotationKiHasChanged() || rotationKdHasChanged();
        }
        public static boolean rangePIDHasChanged() {
            return rangeKpHasChanged() || rangeKiHasChanged() || rangeKdHasChanged();
        }
    }

    /**
     * Call this once during robot initialization to create all Shuffleboard tabs
     */
    public static void initializeAll() {
        if (!initialized) {
            ModelA.initialize();
            ModelB.initialize();
            // ModelC.initialize();
            // ModelD.initialize();
            initialized = true;
        }
    }
}

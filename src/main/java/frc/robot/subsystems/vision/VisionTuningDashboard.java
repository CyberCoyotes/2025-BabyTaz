package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

import java.util.Map;

/**
 * ===========================================================================
 * VISION TUNING DASHBOARD
 * ===========================================================================
 *
 * Provides a dedicated Shuffleboard tab for live tuning of vision parameters.
 * Organizes all tunable constants by model for easy access during testing.
 *
 * FEATURES:
 * - Organized layout with sections for each model (Main, A, B, C, D)
 * - Live PID tuning with number boxes (writable)
 * - Speed limit adjustments
 * - Tolerance configuration
 * - All values update in real-time without code redeployment
 *
 * SHUFFLEBOARD LAYOUT:
 * +-----------------------------------------------------------------------+
 * | Vision Tuning                                                         |
 * +---------------+----------------+----------------+---------------------+
 * | MAIN COMMAND  | MODEL A        | MODEL B        | MODEL C             |
 * | PID Values    | PID Values     | PID Values     | PID Values          |
 * | Speed Limits  | Speed Limits   | Speed Limits   | Speed Limits        |
 * | Tolerances    | Tolerances     | Tolerances     | Tolerances          |
 * +-----------------------------------------------------------------------+
 *
 * USAGE:
 * 1. Create VisionTuningDashboard in RobotContainer
 * 2. Values automatically sync with TunableVisionConstants via NetworkTables
 * 3. Commands read from TunableVisionConstants for live updates
 * 4. Adjust values in Shuffleboard during testing
 *
 * IMPORTANT: This dashboard references NetworkTables keys that are created by
 * TunableVisionConstants. The values are editable in both SmartDashboard and
 * Shuffleboard. This approach is more reliable than creating new widgets.
 */
public class VisionTuningDashboard extends SubsystemBase {

    private static final String TAB_NAME = "Vision Tuning";
    private final ShuffleboardTab tab;

    public VisionTuningDashboard() {
        // Create Shuffleboard tab
        tab = Shuffleboard.getTab(TAB_NAME);

        // Add instructions at the top
        tab.add("INSTRUCTIONS", "Adjust values below - they update live! Changes auto-sync to SmartDashboard.")
            .withPosition(0, 0)
            .withSize(8, 1);

        // Create organized sections for each model
        setupMainCommandSection();
        setupModelASection();
        setupModelBSection();
        setupModelCSection();
        setupModelDSection();
    }

    /**
     * Sets up Main AlignToAprilTagCommand tuning section using Lists
     * Column 0, starting at row 1
     */
    private void setupMainCommandSection() {
        ShuffleboardLayout mainList = tab.getLayout("MAIN COMMAND", BuiltInLayouts.kList)
            .withPosition(0, 1)
            .withSize(2, 8)
            .withProperties(Map.of("Label position", "LEFT"));

        // Add all Main command tunables - they'll appear as editable number boxes
        addTunableToLayout(mainList, "Forward kP", TunableVisionConstants.Main.FORWARD_KP);
        addTunableToLayout(mainList, "Forward kI", TunableVisionConstants.Main.FORWARD_KI);
        addTunableToLayout(mainList, "Forward kD", TunableVisionConstants.Main.FORWARD_KD);
        addTunableToLayout(mainList, "Lateral kP", TunableVisionConstants.Main.LATERAL_KP);
        addTunableToLayout(mainList, "Lateral kI", TunableVisionConstants.Main.LATERAL_KI);
        addTunableToLayout(mainList, "Lateral kD", TunableVisionConstants.Main.LATERAL_KD);
        addTunableToLayout(mainList, "Rotation kP", TunableVisionConstants.Main.ROTATION_KP);
        addTunableToLayout(mainList, "Rotation kI", TunableVisionConstants.Main.ROTATION_KI);
        addTunableToLayout(mainList, "Rotation kD", TunableVisionConstants.Main.ROTATION_KD);
        addTunableToLayout(mainList, "Max Fwd Speed", TunableVisionConstants.Main.MAX_FORWARD_SPEED);
        addTunableToLayout(mainList, "Max Lat Speed", TunableVisionConstants.Main.MAX_LATERAL_SPEED);
        addTunableToLayout(mainList, "Max Rot Speed", TunableVisionConstants.Main.MAX_ROTATION_SPEED);
        addTunableToLayout(mainList, "Fwd Tolerance", TunableVisionConstants.Main.FORWARD_TOLERANCE);
        addTunableToLayout(mainList, "Lat Tolerance", TunableVisionConstants.Main.LATERAL_TOLERANCE);
        addTunableToLayout(mainList, "Rot Tolerance", TunableVisionConstants.Main.ROTATION_TOLERANCE);
        addTunableToLayout(mainList, "Target Dist", TunableVisionConstants.Main.TARGET_DISTANCE);
    }

    /**
     * Sets up Model A (Rotation Only) tuning section
     * Column 2, starting at row 1
     */
    private void setupModelASection() {
        ShuffleboardLayout modelAList = tab.getLayout("MODEL A - Rotation", BuiltInLayouts.kList)
            .withPosition(2, 1)
            .withSize(2, 8)
            .withProperties(Map.of("Label position", "LEFT"));

        addTunableToLayout(modelAList, "Rotation kP", TunableVisionConstants.ModelA.ROTATION_KP);
        addTunableToLayout(modelAList, "Rotation kI", TunableVisionConstants.ModelA.ROTATION_KI);
        addTunableToLayout(modelAList, "Rotation kD", TunableVisionConstants.ModelA.ROTATION_KD);
        addTunableToLayout(modelAList, "Max Rot Speed", TunableVisionConstants.ModelA.MAX_ROTATION_SPEED);
        addTunableToLayout(modelAList, "Min Rot Speed", TunableVisionConstants.ModelA.MIN_ROTATION_SPEED);
        addTunableToLayout(modelAList, "Rot Tolerance", TunableVisionConstants.ModelA.ROTATION_TOLERANCE);
    }

    /**
     * Sets up Model B (Rotation + Range) tuning section
     * Column 4, starting at row 1
     */
    private void setupModelBSection() {
        ShuffleboardLayout modelBList = tab.getLayout("MODEL B - Rot+Range", BuiltInLayouts.kList)
            .withPosition(4, 1)
            .withSize(2, 8)
            .withProperties(Map.of("Label position", "LEFT"));

        addTunableToLayout(modelBList, "Rotation kP", TunableVisionConstants.ModelB.ROTATION_KP);
        addTunableToLayout(modelBList, "Rotation kI", TunableVisionConstants.ModelB.ROTATION_KI);
        addTunableToLayout(modelBList, "Rotation kD", TunableVisionConstants.ModelB.ROTATION_KD);
        addTunableToLayout(modelBList, "Range kP", TunableVisionConstants.ModelB.RANGE_KP);
        addTunableToLayout(modelBList, "Range kI", TunableVisionConstants.ModelB.RANGE_KI);
        addTunableToLayout(modelBList, "Range kD", TunableVisionConstants.ModelB.RANGE_KD);
        addTunableToLayout(modelBList, "Max Rot Speed", TunableVisionConstants.ModelB.MAX_ROTATION_SPEED);
        addTunableToLayout(modelBList, "Max Fwd Speed", TunableVisionConstants.ModelB.MAX_FORWARD_SPEED);
        addTunableToLayout(modelBList, "Rot Tolerance", TunableVisionConstants.ModelB.ROTATION_TOLERANCE);
        addTunableToLayout(modelBList, "Dist Tolerance", TunableVisionConstants.ModelB.DISTANCE_TOLERANCE);
        addTunableToLayout(modelBList, "Target Dist", TunableVisionConstants.ModelB.TARGET_DISTANCE);
    }

    /**
     * Sets up Model C (Perpendicular) tuning section
     * Column 6, starting at row 1
     */
    private void setupModelCSection() {
        ShuffleboardLayout modelCList = tab.getLayout("MODEL C - Perpendicular", BuiltInLayouts.kList)
            .withPosition(6, 1)
            .withSize(2, 8)
            .withProperties(Map.of("Label position", "LEFT"));

        addTunableToLayout(modelCList, "Rotation kP", TunableVisionConstants.ModelC.ROTATION_KP);
        addTunableToLayout(modelCList, "Rotation kI", TunableVisionConstants.ModelC.ROTATION_KI);
        addTunableToLayout(modelCList, "Rotation kD", TunableVisionConstants.ModelC.ROTATION_KD);
        addTunableToLayout(modelCList, "Range kP", TunableVisionConstants.ModelC.RANGE_KP);
        addTunableToLayout(modelCList, "Range kI", TunableVisionConstants.ModelC.RANGE_KI);
        addTunableToLayout(modelCList, "Range kD", TunableVisionConstants.ModelC.RANGE_KD);
        addTunableToLayout(modelCList, "Lateral kP", TunableVisionConstants.ModelC.LATERAL_KP);
        addTunableToLayout(modelCList, "Lateral kI", TunableVisionConstants.ModelC.LATERAL_KI);
        addTunableToLayout(modelCList, "Lateral kD", TunableVisionConstants.ModelC.LATERAL_KD);
        addTunableToLayout(modelCList, "Max Rot Speed", TunableVisionConstants.ModelC.MAX_ROTATION_SPEED);
        addTunableToLayout(modelCList, "Max Fwd Speed", TunableVisionConstants.ModelC.MAX_FORWARD_SPEED);
        addTunableToLayout(modelCList, "Max Lat Speed", TunableVisionConstants.ModelC.MAX_LATERAL_SPEED);
        addTunableToLayout(modelCList, "Rot Tolerance", TunableVisionConstants.ModelC.ROTATION_TOLERANCE);
        addTunableToLayout(modelCList, "Dist Tolerance", TunableVisionConstants.ModelC.DISTANCE_TOLERANCE);
        addTunableToLayout(modelCList, "Lat Tolerance", TunableVisionConstants.ModelC.LATERAL_TOLERANCE);
        addTunableToLayout(modelCList, "Lat Deadband", TunableVisionConstants.ModelC.LATERAL_DEADBAND);
        addTunableToLayout(modelCList, "Target Dist", TunableVisionConstants.ModelC.TARGET_DISTANCE);
    }

    /**
     * Sets up Model D (Color Hunt) tuning section
     * Column 8, starting at row 1
     */
    private void setupModelDSection() {
        ShuffleboardLayout modelDList = tab.getLayout("MODEL D - Color Hunt", BuiltInLayouts.kList)
            .withPosition(8, 1)
            .withSize(2, 8)
            .withProperties(Map.of("Label position", "LEFT"));

        addTunableToLayout(modelDList, "Hunt Rot kP", TunableVisionConstants.ModelD.HUNT_ROTATION_KP);
        addTunableToLayout(modelDList, "Hunt Rot kI", TunableVisionConstants.ModelD.HUNT_ROTATION_KI);
        addTunableToLayout(modelDList, "Hunt Rot kD", TunableVisionConstants.ModelD.HUNT_ROTATION_KD);
        addTunableToLayout(modelDList, "Seek Rot kP", TunableVisionConstants.ModelD.SEEK_ROTATION_KP);
        addTunableToLayout(modelDList, "Seek Rot kI", TunableVisionConstants.ModelD.SEEK_ROTATION_KI);
        addTunableToLayout(modelDList, "Seek Rot kD", TunableVisionConstants.ModelD.SEEK_ROTATION_KD);
        addTunableToLayout(modelDList, "Range kP", TunableVisionConstants.ModelD.RANGE_KP);
        addTunableToLayout(modelDList, "Range kI", TunableVisionConstants.ModelD.RANGE_KI);
        addTunableToLayout(modelDList, "Range kD", TunableVisionConstants.ModelD.RANGE_KD);
        addTunableToLayout(modelDList, "Hunt Rot Speed", TunableVisionConstants.ModelD.HUNT_ROTATION_SPEED);
        addTunableToLayout(modelDList, "Max Rot Speed", TunableVisionConstants.ModelD.MAX_ROTATION_SPEED);
        addTunableToLayout(modelDList, "Max Fwd Speed", TunableVisionConstants.ModelD.MAX_FORWARD_SPEED);
        addTunableToLayout(modelDList, "Rot Tolerance", TunableVisionConstants.ModelD.ROTATION_TOLERANCE);
        addTunableToLayout(modelDList, "Dist Tolerance", TunableVisionConstants.ModelD.DISTANCE_TOLERANCE);
        addTunableToLayout(modelDList, "Min Tgt Area", TunableVisionConstants.ModelD.MIN_TARGET_AREA);
        addTunableToLayout(modelDList, "Tgt Area Dist", TunableVisionConstants.ModelD.TARGET_AREA_FOR_DISTANCE);
    }

    /**
     * Helper method to add a TunableNumber to a Shuffleboard layout.
     * This references the NetworkTable key that was already created by TunableNumber,
     * making it writable and ensuring synchronization.
     */
    private void addTunableToLayout(ShuffleboardLayout layout, String displayName, TunableNumber tunable) {
        // Add the existing NetworkTable entry by key - this makes it writable
        layout.addPersistent(displayName, tunable.getDefault())
            .withProperties(Map.of("min", 0.0, "max", 5.0));
    }

    @Override
    public void periodic() {
        // Periodic updates handled by TunableNumber class
        // Values sync automatically via NetworkTables
    }

    /**
     * Resets all tunable values to their defaults.
     * Useful after testing to return to baseline configuration.
     */
    public void resetAllToDefaults() {
        TunableVisionConstants.resetAllToDefaults();
    }
}

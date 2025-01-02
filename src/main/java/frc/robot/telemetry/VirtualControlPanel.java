package frc.robot.telemetry;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class VirtualControlPanel {
    private final ShuffleboardTab testTab;
    private final Map<String, GenericEntry> virtualButtons = new HashMap<>();
    private final Map<String, GenericEntry> virtualAxes = new HashMap<>();

    public VirtualControlPanel() {
        testTab = Shuffleboard.getTab("Test Controls");
        setupVirtualControls();
    }

    private void setupVirtualControls() {
        // Driver virtual buttons
        var driverButtons = testTab.getLayout("Driver Buttons", BuiltInLayouts.kGrid)
            .withSize(3, 4)
            .withPosition(0, 0)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 4));

        // Add driver buttons
        virtualButtons.put("A Button", driverButtons.add("A Button", false).getEntry());
        virtualButtons.put("B Button", driverButtons.add("B Button", false).getEntry());
        virtualButtons.put("X Button", driverButtons.add("X Button", false).getEntry());
        virtualButtons.put("Y Button", driverButtons.add("Y Button", false).getEntry());
        virtualButtons.put("Left Bumper", driverButtons.add("Left Bumper", false).getEntry());
        virtualButtons.put("Right Bumper", driverButtons.add("Right Bumper", false).getEntry());

        // Driver virtual axes
        var driverAxes = testTab.getLayout("Driver Axes", BuiltInLayouts.kGrid)
            .withSize(4, 3)
            .withPosition(3, 0)
            .withProperties(Map.of("Number of columns", 2, "Number of rows", 3));

        // Add driver axes with sliders
        virtualAxes.put("Left X", driverAxes.add("Left X", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", -1.0, "Max", 1.0))
            .getEntry());
        virtualAxes.put("Left Y", driverAxes.add("Left Y", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", -1.0, "Max", 1.0))
            .getEntry());
        virtualAxes.put("Right X", driverAxes.add("Right X", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", -1.0, "Max", 1.0))
            .getEntry());
        virtualAxes.put("Right Y", driverAxes.add("Right Y", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", -1.0, "Max", 1.0))
            .getEntry());

        // Triggers as sliders
        virtualAxes.put("Left Trigger", driverAxes.add("Left Trigger", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", 0.0, "Max", 1.0))
            .getEntry());
        virtualAxes.put("Right Trigger", driverAxes.add("Right Trigger", 0.0)
            .withWidget("Number Slider")
            .withProperties(Map.of("Min", 0.0, "Max", 1.0))
            .getEntry());

        // Add Command Scheduler widget
        testTab.add("Scheduler", CommandScheduler.getInstance())
            .withSize(4, 3)
            .withPosition(7, 0);
    }

    public Trigger getVirtualButton(String name) {
        return new Trigger(() -> {
            GenericEntry entry = virtualButtons.get(name);
            return entry != null && entry.getBoolean(false);
        });
    }

    public double getVirtualAxis(String name) {
        GenericEntry entry = virtualAxes.get(name);
        return entry != null ? entry.getDouble(0.0) : 0.0;
    }

    public boolean isVirtualButtonPressed(String name) {
        GenericEntry entry = virtualButtons.get(name);
        return entry != null && entry.getBoolean(false);
    }

    public void setVirtualButton(String name, boolean value) {
        GenericEntry entry = virtualButtons.get(name);
        if (entry != null) {
            entry.setBoolean(value);
        }
    }

    public void setVirtualAxis(String name, double value) {
        GenericEntry entry = virtualAxes.get(name);
        if (entry != null) {
            entry.setDouble(value);
        }
    }
}

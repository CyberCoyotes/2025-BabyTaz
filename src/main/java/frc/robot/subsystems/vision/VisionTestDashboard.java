package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.visiontest.ColorBlobHuntCommand;
import frc.robot.commands.visiontest.PerpendicularAlignCommand;
import frc.robot.commands.visiontest.RotationalAlignCommand;
import frc.robot.commands.visiontest.RotationalRangeAlignCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import java.util.Map;

/**
 * ===========================================================================
 * VISION TEST DASHBOARD
 * ===========================================================================
 *
 * Provides Shuffleboard UI for testing vision alignment models.
 *
 * FEATURES:
 * - Virtual buttons to trigger each vision test model
 * - Real-time status display for each model
 * - Stop All button to immediately halt any running test
 * - Telemetry display for current test values
 *
 * SHUFFLEBOARD LAYOUT:
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │ Vision Tests                                                        │
 * ├─────────────────┬──────────────────┬──────────────────┬────────────┤
 * │ Model A:        │ Model B:         │ Model C:         │ Model D:   │
 * │ Rotation Only   │ Rotation + Range │ Perpendicular    │ Color Hunt │
 * │ [Button]        │ [Button]         │ [Button]         │ [Button]   │
 * ├─────────────────┴──────────────────┴──────────────────┴────────────┤
 * │                        [STOP ALL]                                  │
 * ├─────────────────┬──────────────────┬──────────────────┬────────────┤
 * │ Active Model:   │ Status:          │ TX:              │ Distance:  │
 * │ [text]          │ [text]           │ [number]         │ [number]   │
 * └─────────────────┴──────────────────┴──────────────────┴────────────┘
 *
 * USAGE:
 * 1. Create VisionTestDashboard in RobotContainer
 * 2. Dashboard automatically creates Shuffleboard tab
 * 3. Click buttons in Shuffleboard to trigger tests
 * 4. Monitor telemetry in the same tab
 *
 * BUTTON BEHAVIOR:
 * - Clicking a test button starts that test (cancels any running test)
 * - Button stays "pressed" while test is running
 * - STOP ALL immediately cancels any running test
 */
public class VisionTestDashboard extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightVision vision;

    // Shuffleboard tab and entries
    private final ShuffleboardTab tab;

    // Button entries (writable booleans)
    private final GenericEntry modelAButton;
    private final GenericEntry modelBButton;
    private final GenericEntry modelCButton;
    private final GenericEntry modelDButton;
    private final GenericEntry stopButton;

    // Status display entries
    private final GenericEntry activeModelEntry;
    private final GenericEntry statusEntry;
    private final GenericEntry txEntry;
    private final GenericEntry distanceEntry;
    private final GenericEntry alignedEntry;

    // Commands for each model
    private final Command modelACommand;
    private final Command modelBCommand;
    private final Command modelCCommand;
    private final Command modelDCommand;

    // Currently running command (for tracking)
    private Command currentCommand = null;
    private String currentModelName = "NONE";

    public VisionTestDashboard(CommandSwerveDrivetrain drivetrain, LimelightVision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Create commands
        modelACommand = new RotationalAlignCommand(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model A"))
            .finallyDo(interrupted -> clearActiveModel());

        modelBCommand = new RotationalRangeAlignCommand(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model B"))
            .finallyDo(interrupted -> clearActiveModel());

        modelCCommand = new PerpendicularAlignCommand(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model C"))
            .finallyDo(interrupted -> clearActiveModel());

        modelDCommand = new ColorBlobHuntCommand(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model D"))
            .finallyDo(interrupted -> clearActiveModel());

        // Create Shuffleboard tab
        tab = Shuffleboard.getTab(VisionConstants.Dashboard.TAB_NAME);

        // Create button entries - use toggle buttons for visual feedback
        // Row 0: Test buttons
        modelAButton = tab.add("Model A: Rotation Only", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(0, 0)
            .withSize(2, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"))
            .getEntry();

        modelBButton = tab.add("Model B: Rotation + Range", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(2, 0)
            .withSize(2, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"))
            .getEntry();

        modelCButton = tab.add("Model C: Perpendicular", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(4, 0)
            .withSize(2, 1)
            .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "gray"))
            .getEntry();

        modelDButton = tab.add("Model D: Color Hunt", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(6, 0)
            .withSize(2, 1)
            .withProperties(Map.of("colorWhenTrue", "blue", "colorWhenFalse", "gray"))
            .getEntry();

        // Row 1: Stop button
        stopButton = tab.add("STOP ALL", false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(3, 1)
            .withSize(2, 1)
            .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "darkred"))
            .getEntry();

        // Row 2: Status display
        activeModelEntry = tab.add("Active Model", "NONE")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 2)
            .withSize(2, 1)
            .getEntry();

        statusEntry = tab.add("Status", "IDLE")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(2, 2)
            .withSize(2, 1)
            .getEntry();

        txEntry = tab.add("TX (deg)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(4, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", -30, "max", 30))
            .getEntry();

        distanceEntry = tab.add("Distance (m)", 0.0)
            .withWidget(BuiltInWidgets.kNumberBar)
            .withPosition(6, 2)
            .withSize(2, 1)
            .withProperties(Map.of("min", 0, "max", 3))
            .getEntry();

        // Row 3: Aligned indicator
        alignedEntry = tab.add("Aligned", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 3)
            .withSize(1, 1)
            .withProperties(Map.of("colorWhenTrue", "lime", "colorWhenFalse", "red"))
            .getEntry();

        // Add instructions
        tab.add("Instructions", "Click button to start test. Click STOP or same button to stop.")
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 3)
            .withSize(7, 1);

        // Setup triggers for buttons
        setupButtonTriggers();
    }

    /**
     * Sets up triggers to respond to Shuffleboard button presses.
     * Uses polling in periodic() to detect button state changes.
     */
    private void setupButtonTriggers() {
        // Button triggers are handled in periodic() for reliability
    }

    @Override
    public void periodic() {
        // Poll button states and handle commands
        handleButtonPolling();

        // Update telemetry display
        updateTelemetry();
    }

    /**
     * Polls Shuffleboard buttons and triggers commands accordingly.
     * This is more reliable than using Trigger bindings with NetworkTables.
     */
    private void handleButtonPolling() {
        // Check STOP button first (highest priority)
        if (stopButton.getBoolean(false)) {
            stopAllTests();
            stopButton.setBoolean(false);  // Reset button
            return;
        }

        // Check Model A button
        if (modelAButton.getBoolean(false)) {
            if (currentCommand != modelACommand) {
                startTest(modelACommand, "Model A", modelAButton);
            } else {
                // Toggle off - stop the test
                stopAllTests();
            }
            modelAButton.setBoolean(false);  // Reset toggle
        }

        // Check Model B button
        if (modelBButton.getBoolean(false)) {
            if (currentCommand != modelBCommand) {
                startTest(modelBCommand, "Model B", modelBButton);
            } else {
                stopAllTests();
            }
            modelBButton.setBoolean(false);
        }

        // Check Model C button
        if (modelCButton.getBoolean(false)) {
            if (currentCommand != modelCCommand) {
                startTest(modelCCommand, "Model C", modelCButton);
            } else {
                stopAllTests();
            }
            modelCButton.setBoolean(false);
        }

        // Check Model D button
        if (modelDButton.getBoolean(false)) {
            if (currentCommand != modelDCommand) {
                startTest(modelDCommand, "Model D", modelDButton);
            } else {
                stopAllTests();
            }
            modelDButton.setBoolean(false);
        }
    }

    /**
     * Starts a vision test, canceling any currently running test.
     */
    private void startTest(Command command, String modelName, GenericEntry buttonEntry) {
        // Cancel any running test
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }

        // Reset all button visuals
        resetButtonVisuals();

        // Schedule the new command
        command.schedule();
        currentCommand = command;
        currentModelName = modelName;

        Logger.recordOutput("VisionTestDashboard/StartedTest", modelName);
    }

    /**
     * Stops all running vision tests.
     */
    private void stopAllTests() {
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
        currentCommand = null;
        currentModelName = "NONE";
        resetButtonVisuals();

        // Ensure drivetrain is stopped
        drivetrain.stopDrive();

        Logger.recordOutput("VisionTestDashboard/StoppedAllTests", true);
        statusEntry.setString("STOPPED");
    }

    /**
     * Resets all button toggle states to false.
     */
    private void resetButtonVisuals() {
        modelAButton.setBoolean(false);
        modelBButton.setBoolean(false);
        modelCButton.setBoolean(false);
        modelDButton.setBoolean(false);
    }

    private void setActiveModel(String modelName) {
        currentModelName = modelName;
        activeModelEntry.setString(modelName);
        statusEntry.setString("RUNNING");
    }

    private void clearActiveModel() {
        currentModelName = "NONE";
        currentCommand = null;
        activeModelEntry.setString("NONE");
        statusEntry.setString("IDLE");
        resetButtonVisuals();
    }

    /**
     * Updates the telemetry display with current vision values.
     */
    private void updateTelemetry() {
        // Update active model display
        activeModelEntry.setString(currentModelName);

        // Update vision values
        if (vision.hasTarget()) {
            txEntry.setDouble(vision.getTX());
            distanceEntry.setDouble(vision.getDistanceToCM() / 100.0);  // Convert cm to m
        } else {
            txEntry.setDouble(0.0);
            distanceEntry.setDouble(0.0);
        }

        // Log to AdvantageKit
        Logger.recordOutput("VisionTestDashboard/ActiveModel", currentModelName);
        Logger.recordOutput("VisionTestDashboard/IsRunning", currentCommand != null && currentCommand.isScheduled());
    }

    /**
     * Returns a command to run Model A test (for controller binding).
     */
    public Command getModelACommand() {
        return modelACommand;
    }

    /**
     * Returns a command to run Model B test (for controller binding).
     */
    public Command getModelBCommand() {
        return modelBCommand;
    }

    /**
     * Returns a command to run Model C test (for controller binding).
     */
    public Command getModelCCommand() {
        return modelCCommand;
    }

    /**
     * Returns a command to run Model D test (for controller binding).
     */
    public Command getModelDCommand() {
        return modelDCommand;
    }

    /**
     * Returns a command to stop all tests (for controller binding).
     */
    public Command getStopCommand() {
        return Commands.runOnce(this::stopAllTests);
    }
}

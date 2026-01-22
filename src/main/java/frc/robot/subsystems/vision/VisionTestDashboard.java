package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.visiontest.ModelX_ColorBlobCommand;
import frc.robot.commands.visiontest.ModelY_DistancePerpendicular;
import frc.robot.commands.visiontest.ModelA_Rotation;
import frc.robot.commands.visiontest.ModelB_RotationDistance;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("unused")

/**
 * ===========================================================================
 * VISION TEST DASHBOARD
 * ===========================================================================
 *
 * Provides Elastic Dashboard UI for testing vision alignment models.
 *
 * FEATURES:
 * - NetworkTables boolean entries to trigger each vision test model
 * - Real-time status display for each model
 * - Stop All button to immediately halt any running test
 * - Telemetry display for current test values
 *
 * NETWORKTABLES LAYOUT (Elastic/VisionTest):
 * - Buttons:
 *   - ModelAButton (boolean): Toggle to start/stop Model A
 *   - ModelBButton (boolean): Toggle to start/stop Model B
 *   - ModelCButton (boolean): Toggle to start/stop Model C
 *   - ModelDButton (boolean): Toggle to start/stop Model D
 *   - StopButton (boolean): Stops all running tests
 * - Status:
 *   - ActiveModel (string): Currently running model name
 *   - Status (string): Current alignment state
 *   - TX (double): Horizontal offset in degrees
 *   - Distance (double): Distance to target in meters
 *   - Aligned (boolean): Whether robot is aligned
 *
 * USAGE:
 * 1. Create VisionTestDashboard in RobotContainer
 * 2. Dashboard automatically publishes to NetworkTables under "Elastic/VisionTest"
 * 3. Toggle buttons in Elastic Dashboard to trigger tests
 * 4. Monitor telemetry in the same dashboard
 *
 * BUTTON BEHAVIOR:
 * - Setting a test button to true starts that test (cancels any running test)
 * - Button resets to false automatically
 * - STOP ALL immediately cancels any running test
 */
public class VisionTestDashboard extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Elastic Dashboard NetworkTable
    private final NetworkTable elasticTable;

    // Button entries (writable booleans)
    private final BooleanEntry modelAButton;
    private final BooleanEntry modelBButton;
    private final BooleanEntry modelCButton;
    private final BooleanEntry modelDButton;
    private final BooleanEntry stopButton;

    // Status display entries
    private final StringEntry activeModelEntry;
    private final StringEntry statusEntry;
    private final DoubleEntry txEntry;
    private final DoubleEntry distanceEntry;
    private final BooleanEntry alignedEntry;

    // Commands for each model
    private final Command modelACommand;
    private final Command modelBCommand;
    private final Command modelCCommand;
    private final Command modelDCommand;

    // Currently running command (for tracking)
    private Command currentCommand = null;
    private String currentModelName = "NONE";

    public VisionTestDashboard(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.elasticTable = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("VisionTest");

        // Create commands
        modelACommand = new ModelA_Rotation(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model A"))
            .finallyDo(interrupted -> clearActiveModel());

        modelBCommand = new ModelB_RotationDistance(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model B"))
            .finallyDo(interrupted -> clearActiveModel());

        modelCCommand = new ModelY_DistancePerpendicular(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model C"))
            .finallyDo(interrupted -> clearActiveModel());

        modelDCommand = new ModelX_ColorBlobCommand(drivetrain, vision)
            .beforeStarting(() -> setActiveModel("Model D"))
            .finallyDo(interrupted -> clearActiveModel());

        // Create NetworkTables entries for buttons
        modelAButton = elasticTable.getBooleanTopic("ModelAButton").getEntry(false);
        modelBButton = elasticTable.getBooleanTopic("ModelBButton").getEntry(false);
        modelCButton = elasticTable.getBooleanTopic("ModelCButton").getEntry(false);
        modelDButton = elasticTable.getBooleanTopic("ModelDButton").getEntry(false);
        stopButton = elasticTable.getBooleanTopic("StopButton").getEntry(false);

        // Create NetworkTables entries for status display
        activeModelEntry = elasticTable.getStringTopic("ActiveModel").getEntry("NONE");
        statusEntry = elasticTable.getStringTopic("Status").getEntry("IDLE");
        txEntry = elasticTable.getDoubleTopic("TX").getEntry(0.0);
        distanceEntry = elasticTable.getDoubleTopic("Distance").getEntry(0.0);
        alignedEntry = elasticTable.getBooleanTopic("Aligned").getEntry(false);

        // Initialize default values
        modelAButton.set(false);
        modelBButton.set(false);
        modelCButton.set(false);
        modelDButton.set(false);
        stopButton.set(false);
        activeModelEntry.set("NONE");
        statusEntry.set("IDLE");
        txEntry.set(0.0);
        distanceEntry.set(0.0);
        alignedEntry.set(false);
    }


    @Override
    public void periodic() {
        // Poll button states and handle commands
        handleButtonPolling();

        // Update telemetry display
        updateTelemetry();
    }

    /**
     * Polls NetworkTables button entries and triggers commands accordingly.
     * This is more reliable than using Trigger bindings with NetworkTables.
     */
    private void handleButtonPolling() {
        // Check STOP button first (highest priority)
        if (stopButton.get()) {
            stopAllTests();
            stopButton.set(false);  // Reset button
            return;
        }

        // Check Model A button
        if (modelAButton.get()) {
            if (currentCommand != modelACommand) {
                startTest(modelACommand, "Model A", modelAButton);
            } else {
                // Toggle off - stop the test
                stopAllTests();
            }
            modelAButton.set(false);  // Reset toggle
        }

        // Check Model B button
        if (modelBButton.get()) {
            if (currentCommand != modelBCommand) {
                startTest(modelBCommand, "Model B", modelBButton);
            } else {
                stopAllTests();
            }
            modelBButton.set(false);
        }

        // Check Model C button
        if (modelCButton.get()) {
            if (currentCommand != modelCCommand) {
                startTest(modelCCommand, "Model C", modelCButton);
            } else {
                stopAllTests();
            }
            modelCButton.set(false);
        }

        // Check Model D button
        if (modelDButton.get()) {
            if (currentCommand != modelDCommand) {
                startTest(modelDCommand, "Model D", modelDButton);
            } else {
                stopAllTests();
            }
            modelDButton.set(false);
        }
    }

    /**
     * Starts a vision test, canceling any currently running test.
     */
    private void startTest(Command command, String modelName, BooleanEntry buttonEntry) {
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
   //     drivetrain.stopDrive();

        Logger.recordOutput("VisionTestDashboard/StoppedAllTests", true);
        statusEntry.set("STOPPED");
    }

    /**
     * Resets all button toggle states to false.
     */
    private void resetButtonVisuals() {
        modelAButton.set(false);
        modelBButton.set(false);
        modelCButton.set(false);
        modelDButton.set(false);
    }

    private void setActiveModel(String modelName) {
        currentModelName = modelName;
        activeModelEntry.set(modelName);
        statusEntry.set("RUNNING");
    }

    private void clearActiveModel() {
        currentModelName = "NONE";
        currentCommand = null;
        activeModelEntry.set("NONE");
        statusEntry.set("IDLE");
        resetButtonVisuals();
    }

    /**
     * Updates the telemetry display with current vision values.
     */
    private void updateTelemetry() {
        // Update active model display
        activeModelEntry.set(currentModelName);

        // Update status from state machine
        statusEntry.set(vision.getAlignmentState().name());

        // Update aligned indicator from state machine
        alignedEntry.set(vision.isAligned());

        // Update vision values
        if (vision.hasTarget()) {
            txEntry.set(vision.getTX());
            distanceEntry.set(vision.getDistanceToCM() / 100.0);  // Convert cm to m
        } else {
            txEntry.set(0.0);
            distanceEntry.set(0.0);
        }

        // Log to AdvantageKit
        Logger.recordOutput("VisionTestDashboard/ActiveModel", currentModelName);
        Logger.recordOutput("VisionTestDashboard/IsRunning", currentCommand != null && currentCommand.isScheduled());
        Logger.recordOutput("VisionTestDashboard/VisionMode", vision.getVisionMode().name());
        Logger.recordOutput("VisionTestDashboard/AlignmentState", vision.getAlignmentState().name());
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

package frc.robot.controls;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlsTelemetry {
    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final ShuffleboardTab controlsTab;

    public ControlsTelemetry(CommandXboxController driver, CommandXboxController operator) {
        this.driver = driver;
        this.operator = operator;
        this.controlsTab = Shuffleboard.getTab("Controls");
        
        configureShuffleboard();
    }

    private void configureShuffleboard() {
        // Driver Controls
        var driverLayout = controlsTab.getLayout("Driver Controls", "Grid Layout")
            .withSize(4, 5)
            .withPosition(0, 0);
            
        // Add driver joystick values
        driverLayout.addDouble("Left X", () -> driver.getLeftX());
        driverLayout.addDouble("Left Y", () -> driver.getLeftY());
        driverLayout.addDouble("Right X", () -> driver.getRightX());
        driverLayout.addDouble("Right Y", () -> driver.getRightY());
        
        // Add driver trigger values
        driverLayout.addDouble("Left Trigger", () -> driver.getLeftTriggerAxis());
        driverLayout.addDouble("Right Trigger", () -> driver.getRightTriggerAxis());
        
        // Add driver button states
        driverLayout.addBoolean("A Button", () -> driver.a().getAsBoolean());
        driverLayout.addBoolean("B Button", () -> driver.b().getAsBoolean());
        driverLayout.addBoolean("X Button", () -> driver.x().getAsBoolean());
        driverLayout.addBoolean("Y Button", () -> driver.y().getAsBoolean());
        driverLayout.addBoolean("Left Bumper", () -> driver.leftBumper().getAsBoolean());
        driverLayout.addBoolean("Right Bumper", () -> driver.rightBumper().getAsBoolean());
        driverLayout.addInteger("POV", () -> driver.getHID().getPOV());

        // Operator Controls
        var operatorLayout = controlsTab.getLayout("Operator Controls", "Grid Layout")
            .withSize(4, 5)
            .withPosition(4, 0);
            
        // Add operator joystick values
        operatorLayout.addDouble("Left X", () -> operator.getLeftX());
        operatorLayout.addDouble("Left Y", () -> operator.getLeftY());
        operatorLayout.addDouble("Right X", () -> operator.getRightX());
        operatorLayout.addDouble("Right Y", () -> operator.getRightY());
        
        // Add operator trigger values
        operatorLayout.addDouble("Left Trigger", () -> operator.getLeftTriggerAxis());
        operatorLayout.addDouble("Right Trigger", () -> operator.getRightTriggerAxis());
        
        // Add operator button states
        operatorLayout.addBoolean("A Button", () -> operator.a().getAsBoolean());
        operatorLayout.addBoolean("B Button", () -> operator.b().getAsBoolean());
        operatorLayout.addBoolean("X Button", () -> operator.x().getAsBoolean());
        operatorLayout.addBoolean("Y Button", () -> operator.y().getAsBoolean());
        operatorLayout.addBoolean("Left Bumper", () -> operator.leftBumper().getAsBoolean());
        operatorLayout.addBoolean("Right Bumper", () -> operator.rightBumper().getAsBoolean());
        operatorLayout.addInteger("POV", () -> operator.getHID().getPOV());

        // Add Command Scheduler data
        controlsTab.add("Scheduler", CommandScheduler.getInstance())
            .withSize(4, 3)
            .withPosition(8, 0);
    }

    public void updateTelemetry() {
        // This method can be called periodically to update any non-lambda telemetry
        // Currently all our telemetry uses lambdas which auto-update
    }
}

package frc.robot.controls;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlsTelemetry {
    private final CommandXboxController driver;
    private final CommandXboxController operator;
    private final NetworkTable elasticTable;

    public ControlsTelemetry(CommandXboxController driver, CommandXboxController operator) {
        this.driver = driver;
        this.operator = operator;
        this.elasticTable = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("Controls");
    }

    public void updateTelemetry() {
        // Update driver controls to NetworkTables
        NetworkTable driverTable = elasticTable.getSubTable("Driver");
        driverTable.getEntry("LeftX").setDouble(driver.getLeftX());
        driverTable.getEntry("LeftY").setDouble(driver.getLeftY());
        driverTable.getEntry("RightX").setDouble(driver.getRightX());
        driverTable.getEntry("RightY").setDouble(driver.getRightY());
        driverTable.getEntry("LeftTrigger").setDouble(driver.getLeftTriggerAxis());
        driverTable.getEntry("RightTrigger").setDouble(driver.getRightTriggerAxis());
        driverTable.getEntry("AButton").setBoolean(driver.a().getAsBoolean());
        driverTable.getEntry("BButton").setBoolean(driver.b().getAsBoolean());
        driverTable.getEntry("XButton").setBoolean(driver.x().getAsBoolean());
        driverTable.getEntry("YButton").setBoolean(driver.y().getAsBoolean());
        driverTable.getEntry("LeftBumper").setBoolean(driver.leftBumper().getAsBoolean());
        driverTable.getEntry("RightBumper").setBoolean(driver.rightBumper().getAsBoolean());
        driverTable.getEntry("POV").setInteger(driver.getHID().getPOV());

        // Update operator controls to NetworkTables
        NetworkTable operatorTable = elasticTable.getSubTable("Operator");
        operatorTable.getEntry("LeftX").setDouble(operator.getLeftX());
        operatorTable.getEntry("LeftY").setDouble(operator.getLeftY());
        operatorTable.getEntry("RightX").setDouble(operator.getRightX());
        operatorTable.getEntry("RightY").setDouble(operator.getRightY());
        operatorTable.getEntry("LeftTrigger").setDouble(operator.getLeftTriggerAxis());
        operatorTable.getEntry("RightTrigger").setDouble(operator.getRightTriggerAxis());
        operatorTable.getEntry("AButton").setBoolean(operator.a().getAsBoolean());
        operatorTable.getEntry("BButton").setBoolean(operator.b().getAsBoolean());
        operatorTable.getEntry("XButton").setBoolean(operator.x().getAsBoolean());
        operatorTable.getEntry("YButton").setBoolean(operator.y().getAsBoolean());
        operatorTable.getEntry("LeftBumper").setBoolean(operator.leftBumper().getAsBoolean());
        operatorTable.getEntry("RightBumper").setBoolean(operator.rightBumper().getAsBoolean());
        operatorTable.getEntry("POV").setInteger(operator.getHID().getPOV());
    }
}

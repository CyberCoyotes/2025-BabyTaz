package frc.robot.subsystems.clockwork;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.clockwork.ClockworkDriveConstants;
import frc.robot.subsystems.led.LEDState;
import frc.robot.subsystems.led.LEDSubsystem;

public class ClockworkVisionSubsystem extends SubsystemBase {
    private VisionState currentState = VisionState.NO_TARGET;
    private final LEDSubsystem leds;

    public ClockworkVisionSubsystem(LEDSubsystem leds) {
        this.leds = leds;
        LimelightHelpers.setLEDMode_ForceOn("marvin"); // Turn on Limelight LEDs
    }

    @Override 
    public void periodic() {
        updateVisionState();
        updateTelemetry();
    }

    private void updateVisionState() {
        boolean hasTarget = LimelightHelpers.getTV("marvin");
        double targetArea = LimelightHelpers.getTA("marvin");

        // Update vision state based on target visibility
        if (!hasTarget || targetArea < ClockworkDriveConstants.MIN_TARGET_AREA) {
            currentState = VisionState.NO_TARGET;
        } else {
            currentState = VisionState.TARGET_VISIBLE;
        }
        
        // Update LEDs based on vision state
        // updateLEDs();
    }

    private void updateLEDs() {
        if (leds != null) {
            switch (currentState) {
                case TARGET_VISIBLE:
                    leds.setState(LEDState.TARGET_VISIBLE);
                    break;
                case NO_TARGET:
                    leds.setState(LEDState.NO_TARGET); 
                    break;
            }
        }
    }

    public boolean hasTarget() {
        return currentState != VisionState.NO_TARGET;
    }

    public double getHorizontalOffset() {
        return LimelightHelpers.getTX("marvin") * 
               (ClockworkDriveConstants.LIMELIGHT_MOUNTED_FRONT ? 1.0 : -1.0);
    }

    public double getVerticalOffset() {
        return LimelightHelpers.getTY("marvin"); // FIXME
    }

    private void updateTelemetry() {
        SmartDashboard.putString("Vision/State", currentState.toString());
        SmartDashboard.putNumber("Vision/HorizontalOffset", getHorizontalOffset());
        SmartDashboard.putNumber("Vision/VerticalOffset", getVerticalOffset());
        SmartDashboard.putBoolean("Vision/HasTarget", hasTarget());

        /*
        // Create a Shuffleboard Tab for Clockwork Vision
        ShuffleboardTab tab = Shuffleboard.getTab("Clockwork Vision");
        tab.add("State", currentState.toString())
            .withWidget("String Box");
        tab.add("Horizontal Offset", getHorizontalOffset())
            .withWidget("Number Bar");
        tab.add("Vertical Offset", getVerticalOffset())
            .withWidget("Number Bar");
        tab.add("Has Target", hasTarget())
            .withWidget("Boolean Box");
        */
    }

    public enum VisionState {
        NO_TARGET,
        TARGET_VISIBLE
    }
}
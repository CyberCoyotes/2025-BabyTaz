
// LEDSubsystem.java
package frc.robot.io.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDSubsystem extends SubsystemBase {
    private final LEDIO io;
    private final LEDIO.LEDIOInputs inputs = new LEDIO.LEDIOInputs();
    private boolean isAnimating = false;
    
    public LEDSubsystem(LEDIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        // Log data to SmartDashboard
        SmartDashboard.putString("LED/State", inputs.currentState.toString());
        SmartDashboard.putNumber("LED/Battery Voltage", inputs.vbat);
        SmartDashboard.putNumber("LED/5V Rail", inputs.v5);
        SmartDashboard.putBoolean("LED/Animating", isAnimating);
    }

    public void setState(LEDState state) {
        if (isAnimating) {
            clearAnimation();
        }
        io.setState(state);
    }

    public void setAnimation(Animation animation) {
        io.setAnimation(animation);
        isAnimating = true;
    }

    public void clearAnimation() {
        io.clearAnimation();
        isAnimating = false;
        // Restore the last known state
        io.setState(inputs.currentState);
    }

    public LEDState getCurrentState() {
        return inputs.currentState;
    }

    public boolean isAnimating() {
        return isAnimating;
    }
}
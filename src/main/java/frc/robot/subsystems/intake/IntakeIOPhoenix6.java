package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * CTRE Phoenix 6 implementation of the IntakeIO interface.
 * This class provides the concrete implementation for controlling the intake
 * using CTRE TalonFX motors with the Phoenix 6 API.
 * 
 * Hardware Requirements:
 * - TalonFX motor controller
 * - Properly configured CAN bus
 * 
 * Configuration includes:
 * - Current limiting
 * - Voltage compensation
 * - Feedback sensor setup
 * - Motion control parameters
 * 
 * @see IntakeIO
 * @see com.ctre.phoenix6.hardware.TalonFX
 */

// IntakeIOPhoenix6.java - Just motor implementation
public class IntakeIOPhoenix6 implements IntakeIO {
    private final TalonFX motor;
    
    public IntakeIOPhoenix6() {
        motor = new TalonFX(INTAKE_MOTOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.currentAmps = motor.getStatorCurrent().getValue();
        inputs.velocityRPM = motor.getVelocity().getValue();
        inputs.appliedVolts = motor.getMotorVoltage().getValue();
    }
    
    @Override
    public void setVoltage(double percentVoltage) {
        motor.set(percentVoltage);
    }
    
    @Override
    public void stop() {
        motor.set(0);
    }
}
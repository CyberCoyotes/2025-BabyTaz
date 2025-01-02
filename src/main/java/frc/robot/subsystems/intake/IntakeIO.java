package frc.robot.subsystems.intake;

/**
 * Interface for hardware abstraction of the intake mechanism.
 * This interface defines the contract for different hardware implementations
 * of the intake system. It allows for easy swapping between different motor
 * controllers
 * or creating mock implementations for testing.
 * 
 * The interface uses an inputs class to track state and supports periodic
 * updates
 * through the updateInputs method.
 * 
 * Implementations should handle:
 * - Motor control for the intake mechanism
 * - Sensor feedback (current, velocity, position)
 * - Hardware-specific configuration
 */

// import org.littletonrobotics.junction.AutoLog;
    /**
     * Data structure for intake inputs (sensors, encoders, etc.).
     * This class is used to periodically update and log the state of the intake
     * system.
     */

    // Instructions for adding AdvantageKit's logging framework
    // https://claude.ai/chat/53074f81-fd1c-4e36-923a-1d705b13ab38

    // Official documentation
    // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template
    // FIXME @AutoLog

// IntakeIO.java - Keep this focused just on motor control
public interface IntakeIO {
    public static final int INTAKE_MOTOR_ID = 20;
    
    // @AutoLog
    public static class IntakeIOInputs {
        public double currentAmps = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public boolean isAtTarget = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    default void setVoltage(double percentVoltage) {}
    default void stop() {}
}
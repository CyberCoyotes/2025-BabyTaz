package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ElevatorConfigs {
    // Hardware Configuration
    public static final int MOTOR_ID = 1;
    public static final String CAN_BUS = "rio";
    
    // Control Settings
    public static final double MAX_VOLTAGE = 11.0;
    public static final double MANUAL_RATE = 0.3;
    public static final double POSITION_TOLERANCE = 0.5; // rotations
    
    // Current Limits
    public static final double CURRENT_LIMIT = 40.0;
    public static final boolean CURRENT_LIMIT_ENABLED = true;
    
    // PID Gains
    public static final double kP = 24.0;
    public static final double kI = 0.0;
    public static final double kD = 0.1;
    public static final double kS = 0.25;  // Static friction compensation
    public static final double kV = 0.12;  // Velocity feedforward
    public static final double kG = 0.0;   // Gravity feedforward
    
    // Physical Configuration
    public static final double GEAR_RATIO = 10.0;  // Motor rotations per mechanism rotation
    public static final double MAX_HEIGHT = 40.0;  // Maximum height in rotations
    
    // State positions in rotations
    public static final class Positions {
        public static final double START = 0.0;
        public static final double L1 = 10.0;
        public static final double L2 = 20.0;
        public static final double L3 = 30.0;
        public static final double L4 = 40.0;
    }
    
    /**
     * Creates and returns a TalonFX configuration with all settings configured
     */
    public static TalonFXConfiguration getTalonFXConfig() {
        var config = new TalonFXConfiguration();
        
        // Configure PID gains
        var slot0 = config.Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        slot0.kS = kS;
        slot0.kV = kV;
        slot0.kG = kG;
        
        // Configure current limits
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = CURRENT_LIMIT_ENABLED;
        
        // Configure motor output
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Configure feedback
        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        
        return config;
    }
}
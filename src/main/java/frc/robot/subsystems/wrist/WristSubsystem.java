package frc.robot.subsystems.wrist;

import java.util.Map;

// Necessary Imports
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration; 
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class WristSubsystem extends SubsystemBase {
    private final TalonFX wristMotor;
    private final PositionVoltage positionRequest;
    
    // Constants
    private static final int WRIST_MOTOR_ID = 20; // Update with your CAN ID
    private static final double WRIST_GEAR_RATIO = 75.0; // Update with your gear ratio
    private static final double ENCODER_TO_MECHANISM_RATIO = 1.0; // Update if encoder gearing differs
    
    // Soft limits in rotations
    private static final double MIN_POSITION = -0.25; // Adjust based on mechanism 
    private static final double MAX_POSITION = 0.25;  // Adjust based on mechanism

    // Constructor
    // FIXME - Update with your motor configuration and move to a separate file

    public WristSubsystem() {
        wristMotor = new TalonFX(WRIST_MOTOR_ID, "canivore");  // Update canbus name
        positionRequest = new PositionVoltage(0).withSlot(0);
        
        configureMotor();
    }

    private void configureMotor() {
        var motorConfig = new TalonFXConfiguration();

        // Configure motor output
        var motorOutputConfigs = motorConfig.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configure feedback/scaling
        var feedback = motorConfig.Feedback;
        feedback.SensorToMechanismRatio = WRIST_GEAR_RATIO * ENCODER_TO_MECHANISM_RATIO;
        
        // Configure soft limits 
        var softLimits = motorConfig.SoftwareLimitSwitch;
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = MAX_POSITION;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = MIN_POSITION;

        // Configure PID gains
        var slot0 = motorConfig.Slot0;
        slot0.kP = 80.0; // Tune these values
        slot0.kI = 0;
        slot0.kD = 0.8;
        slot0.kG = 0.15; // Gravity feedforward - tune this
        slot0.kV = 0.12; // Calculate: 12V / free speed
        slot0.kS = 0.25; // Tune: minimum voltage to move

        wristMotor.getConfigurator().apply(motorConfig);
    }

    // Add methods to control the wrist

    public void setPosition(double targetPositionRotations) {
        wristMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public double getPosition() {
        // Claude suggestion
        // return wristMotor.getPosition().getValue();

        // Needs a double to work with setPosition()
        return wristMotor.getPosition().getValueAsDouble();
    }

    public boolean atPosition() {
        return Math.abs(wristMotor.getClosedLoopError().getValue()) < 0.02; // Tune threshold
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    @Override
    public void periodic() {
        // Add telemetry
        SmartDashboard.putNumber("Wrist/Position", getPosition());
        SmartDashboard.putNumber("Wrist/Target", positionRequest.Position);
        SmartDashboard.putBoolean("Wrist/AtPosition", atPosition());

        // Add telemetry to a Shuffleboard widget
        ShuffleboardTab tabWrist = Shuffleboard.getTab("Wrist");

        
        tabWrist.add("Position", getPosition())
            .withWidget(BuiltInWidgets.kGraph);
        tabWrist.add("Target", positionRequest.Position)
            .withWidget(BuiltInWidgets.kGraph);
        tabWrist.add("AtPosition", atPosition())
            .withWidget(BuiltInWidgets.kBooleanBox);
        

        /* TODO Add tunable parameters
        tabWrist.addNumber("Min Pose", () -> MIN_POSITION)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0)); 

        tabWrist.addNumber("Max Pose", () -> MAX_POSITION)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", -1.0, "max", 1.0)); 
        */

        // Tunable Example
        // tabWrist.addNumber("Max Velocity", () -> Constraints.MAX_VELOCITY)
        //    .withWidget(BuiltInWidgets.kNumberSlider)
        //    .withProperties(Map.of("min", 0.0, "max", 2.0));
    }
}
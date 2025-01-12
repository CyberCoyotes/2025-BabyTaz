package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX motor;
    private final PositionVoltage positionRequest;
    private final VoltageOut percentRequest;
    private ElevatorState currentState = ElevatorState.START;

    public ElevatorSubsystem() {
        motor = new TalonFX(ElevatorConfigs.MOTOR_ID, ElevatorConfigs.CAN_BUS);
        configureMotor();
        
        positionRequest = new PositionVoltage(0).withSlot(0);
        percentRequest = new VoltageOut(0);
    }

    private void configureMotor() {
        // Apply the pre-configured settings
        motor.getConfigurator().apply(ElevatorConfigs.getTalonFXConfig());
        
        // Zero the encoder on startup
        motor.setPosition(0);
    }

    public void setPosition(ElevatorState state) {
        currentState = state;
        motor.setControl(positionRequest.withPosition(state.position));
    }

    public void manualControl(double speed) {
        double voltage = speed * ElevatorConfigs.MAX_VOLTAGE * ElevatorConfigs.MANUAL_RATE;
        motor.setControl(percentRequest.withOutput(voltage));
    }

    public boolean atPosition() {
        // FIXME before implementing this method, we need to add a getPosition() method to the TalonFX class
        // double currentPos = motor.getPosition().getValue();
        // return Math.abs(currentPos - currentState.position) < ElevatorConfigs.POSITION_TOLERANCE;
        return false;

    }

    public ElevatorState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
        // FIXME Log data to SmartDashboard
        // SmartDashboard.putNumber("Elevator/Position", motor.getPosition().getValue());
        // SmartDashboard.putString("Elevator/State", currentState.toString());
        // SmartDashboard.putNumber("Elevator/Current", motor.getStatorCurrent().getValue());
        // SmartDashboard.putNumber("Elevator/Voltage", motor.getMotorVoltage().getValue());
    }
}

/* TODO Add to Robot Container at a later Date */
//
// public class RobotContainer {
//     private final ElevatorSubsystem elevator = new ElevatorSubsystem();
//     private final CommandXboxController operatorController = new CommandXboxController(1);

//     public RobotContainer() {
//         configureBindings();
//     }

//     private void configureBindings() {
//         // Manual control on left joystick Y-axis
//         elevator.setDefaultCommand(
//             new ManualElevatorControl(
//                 elevator, 
//                 () -> -operatorController.getLeftY()
//             )
//         );

//         // Preset positions on buttons
//         operatorController.a().onTrue(new SetElevatorPosition(elevator, ElevatorState.START));
//         operatorController.b().onTrue(new SetElevatorPosition(elevator, ElevatorState.L1));
//         operatorController.x().onTrue(new SetElevatorPosition(elevator, ElevatorState.L2));
//         operatorController.y().onTrue(new SetElevatorPosition(elevator, ElevatorState.L3));
//         operatorController.rightBumper().onTrue(new SetElevatorPosition(elevator, ElevatorState.L4));
//     }
// } 

// See reference https://claude.ai/chat/502fe7aa-adaf-4f6d-8747-57013296c25d
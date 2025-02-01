package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// https://claude.ai/chat/420a3d4c-6d25-4de7-8930-f6be5ae112ec
// Includes more modern example if wanting to persue this further

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static edu.wpi.first.wpilibj2.command.Commands.print;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor;

    // Constants for motor speed
    private static final double CLOCKWISE = 0.5;
    public static final double COUNTERCLOCKWISE = CLOCKWISE * -1;

    int motorCANID = 27;

    public TurretSubsystem(int motorCANID) {
        motor = new WPI_TalonSRX(motorCANID);
        configureMotor();
    }

    private void configureMotor() {
        motor.configFactoryDefault();
        motor.setInverted(false);
        // motor.setSensorPhase(false);
        motor.configVoltageCompSaturation(12);
        motor.enableVoltageCompensation(true);
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configPeakOutputForward(0.7);  // Limit max output
        motor.configPeakOutputReverse(-0.7);
    }
    public void setTurretSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    // Factory Method will be used to create the command?
    // Implement a command to turn the turret clockwise.
    // The command should run until a certain condition is met, and then stop the motor.
    public Command turnClockwise() {
        return Commands.run(() -> setTurretSpeed(CLOCKWISE))
        // .until(() -> /* some condition */)
        .withTimeout(1)
        .finallyDo(interrupted -> stopMotor());    
    }

    public Command turnCounterClockwise() {
        return Commands.run(() -> setTurretSpeed(COUNTERCLOCKWISE))
        // .until(() -> /* some condition */)
        .withTimeout(0.75)
        .finallyDo(interrupted -> stopMotor());    
    }

    public Command turnClockwiseNonVoid() {
        return new Command() {
            @Override
            public void initialize() {
                motor.set(ControlMode.PercentOutput, CLOCKWISE);
                // Add a print command to print the speed of the motor
                System.out.println("Turret speed: " + motor.getMotorOutputPercent());
            }

            @Override
            public boolean isFinished() {
                return true; // Command ends immediately after initialization
            }
        };
    }

    public void turnCounterClockwise(double speed) {
        motor.set(ControlMode.PercentOutput, COUNTERCLOCKWISE);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        print("Turret speed: " + motor.getMotorOutputPercent());

    }

}
package frc.robot.turret;

// https://claude.ai/chat/420a3d4c-6d25-4de7-8930-f6be5ae112ec
// Includes more modern example if wanting to persue this further

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor;

    // Constants for motor speed
    private static final double CLOCKWISE_SPEED = 0.5;
    private static final double COUNTERCLOCKWISE_SPEED = -0.5;

    int motorCANID = 27;

    public TurretSubsystem(int motorCANID) {
        motor = new WPI_TalonSRX(motorCANID);
    }

    public void setMotorSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}


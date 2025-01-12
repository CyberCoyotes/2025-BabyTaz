package frc.robot.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor;
    private final Timer timer;

    // Constants for motor speed
    private static final double CLOCKWISE_SPEED = 0.5;
    private static final double COUNTERCLOCKWISE_SPEED = -0.5;
    int motorCANID = 27;

    public TurretSubsystem(int motorCANID) {
        motor = new WPI_TalonSRX(motorCANID);
        timer = new Timer();
    }

    public void turnCounterClockwiseTimer() {
        timer.reset();
        timer.start();
        while (timer.get() < 1.0) {
            motor.set(ControlMode.PercentOutput, COUNTERCLOCKWISE_SPEED);
        }
        motor.set(ControlMode.PercentOutput, 0);
        timer.stop();
    }

    public void turnClockwiseTimer() {
        timer.reset();
        timer.start();
        while (timer.get() < 1.0) {
            motor.set(ControlMode.PercentOutput, CLOCKWISE_SPEED);
        }
        motor.set(ControlMode.PercentOutput, 0);
        timer.stop();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

} // end of class TurretSubsystem
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class TurnTurretClockwise extends Command {
    private final TurretSubsystem motorSubsystem;
    private final Timer timer = new Timer();

    public TurnTurretClockwise(TurretSubsystem subsystem) {
        motorSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        motorSubsystem.setMotorSpeed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stopMotor();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0);
    }
}
package frc.robot.experimental;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class MoveElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final DoubleSupplier speedSupplier;

    public MoveElevator(ElevatorSubsystem elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.manualControl(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        elevator.manualControl(0);
    }
}
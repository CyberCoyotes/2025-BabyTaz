package frc.robot.experimental;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;

public class SetElevatorPosition extends Command {
    private final ElevatorSubsystem elevator;
    private final ElevatorState targetState;

    public SetElevatorPosition(ElevatorSubsystem elevator, ElevatorState state) {
        this.elevator = elevator;
        this.targetState = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetState);
    }

    @Override
    public boolean isFinished() {
        return elevator.atPosition();
    }
}
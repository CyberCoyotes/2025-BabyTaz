package frc.robot.subsystems.elevator;

public enum ElevatorState {
    START(ElevatorConfigs.Positions.START),
    L1(ElevatorConfigs.Positions.L1),
    L2(ElevatorConfigs.Positions.L2),
    L3(ElevatorConfigs.Positions.L3),
    L4(ElevatorConfigs.Positions.L4);

    public final double position;

    ElevatorState(double position) {
        this.position = position;
    }
}
package frc.robot.subsystems.arm;

public interface ArmIO {

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setLeds(boolean on) {}

    // @AutoLog
    class IntakeIOInputs { 

    }

}

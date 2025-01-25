package frc.robot.subsystems.wrist;

public interface ArmIO {

    default void updateInputs(ArmIOInputs inputs) {}

    // default void setLeds(boolean on) {}

    // @AutoLog
    class ArmIOInputs { 

    }

}

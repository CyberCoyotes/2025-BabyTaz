package frc.robot.subsystems.arm;

public interface ArmIO {

    default void updateInputs(ArmIOInputs inputs) {}

    // default void setLeds(boolean on) {}

    // @AutoLog
    class ArmIOInputs { 

    }

}

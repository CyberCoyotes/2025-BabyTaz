package frc.robot.subsystems.intake;

public interface IntakeIO {

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setLeds(boolean on) {}

    // @AutoLog
    class IntakeIOInputs { 

    }

}

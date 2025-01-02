package frc.robot.subsystems.intake;

public interface IntakeSensorIO {
    public static final int TOF_SENSOR_ID = 21;
    
    // @AutoLog
    public static class IntakeSensorIOInputs {
        public double tofDistanceMillimeters = 0.0;
        public boolean hasGamePiece = false;
    }
    
    default void updateInputs(IntakeSensorIOInputs inputs) {}
}
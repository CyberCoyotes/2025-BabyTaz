package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TOFSubsystem extends SubsystemBase {
    private final TimeOfFlight tof;
    private static final int SENSOR_RANGE_MM = 5000; // 5m max range
    private static final int MIN_RANGE_MM = 30; // Minimum valid range
    
    public TOFSubsystem() {
        tof = new TimeOfFlight(0); // TODO double check
        // Configure for long range mode
        tof.setRangingMode(TimeOfFlight.RangingMode.Long, 24);
        // Sample time of 50ms
        tof.setRangeOfInterest(8, 8, 12, 12);
    }

    @Override
    public void periodic() {
        // Log distance data
        SmartDashboard.putNumber("TOF/Distance_mm", getRawDistance());
        SmartDashboard.putNumber("TOF/Distance_m", getDistanceMeters());
        SmartDashboard.putBoolean("TOF/Valid", isRangeValid());
    }

    public double getRawDistance() {
        return tof.getRange();
    }

    public double getDistanceMeters() {
        return getRawDistance() / 1000.0;
    }

    public boolean isRangeValid() {
        double range = getRawDistance();
        return range > MIN_RANGE_MM && range < SENSOR_RANGE_MM;
    }
}
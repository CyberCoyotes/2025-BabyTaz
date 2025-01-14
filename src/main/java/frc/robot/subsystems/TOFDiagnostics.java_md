package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;

public class TOFDiagnostics extends SubsystemBase {
    private final TimeOfFlight tof;
    private final String name;

    // Constants for range validation 
    private static final int MIN_RANGE_MM = 30;
    private static final int MAX_RANGE_MM = 5000;

    public TOFDiagnostics(int canID, String name) {
        this.tof = new TimeOfFlight(canID);
        this.name = name;
        
        // Configure initial settings
        configureDefaults();
    }

    private void configureDefaults() {
        // Set to long range mode with 24ms timing budget
        tof.setRangingMode(RangingMode.Long, 24);
        
        // Set ROI (Region of Interest) - helps with accuracy
        tof.setRangeOfInterest(8, 8, 12, 12);
    }

    @Override
    public void periodic() {
        // Log all relevant data for diagnostics
        SmartDashboard.putNumber(name + "/Range_Raw_mm", tof.getRange());
        SmartDashboard.putNumber(name + "/Range_Meters", getDistanceMeters());
        SmartDashboard.putBoolean(name + "/Is_Valid", isRangeValid());
        SmartDashboard.putString(name + "/Status", getStatusString());
        // SmartDashboard.putNumber(name + "/Temperature_C", tof.getSensorTemperature());
        SmartDashboard.putNumber(name + "/Signal_Strength", getSignalStrength());
        SmartDashboard.putString(name + "/Ranging_Mode", tof.getRangingMode().toString());
    }

    public double getDistanceMeters() {
        return tof.getRange() / 1000.0; // Convert mm to meters
    }

    public boolean isRangeValid() {
        Status status = tof.getStatus();
        if (status != Status.Valid) {
            return false;
        }

        double range = tof.getRange();
        return range >= MIN_RANGE_MM && range <= MAX_RANGE_MM;
    }

    private String getStatusString() {
        Status status = tof.getStatus();
        switch (status) {
            case Valid:
                return "Valid";
            case Sigma_Fail:
                return "Signal Error";
            case Signal_Fail:
                return "Weak Signal";
            case Phase_Fail:
                return "Phase Error";
            case OutOfBounds:
                return "Out of Range";
            case Wrap_Target_Fail:
                return "Multiple Targets";
            case Not_Found:
                return "No Target";
            case No_Update:
                return "No Update";
            default:
                return "Unknown Error";
        }
    }

    private int getSignalStrength() {
        // Returns relative signal strength (0-100%)
        return tof.getSigStrengthPercent();
    }

    public void setShortRange() {
        tof.setRangingMode(RangingMode.Short, 12);
    }

    public void setMediumRange() {
        tof.setRangingMode(RangingMode.Medium, 16);
    }

    public void setLongRange() {
        tof.setRangingMode(RangingMode.Long, 24);
    }
}
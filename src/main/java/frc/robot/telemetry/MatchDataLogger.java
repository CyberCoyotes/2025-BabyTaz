package frc.robot.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;

/**
 * Logs match-specific data to NetworkTables for Elastic Dashboard.
 * Tracks match number, type, event name, alliance color, and timing information.
 */
public class MatchDataLogger {
    private final NetworkTable matchTable;
    private double matchStartTime;

    /**
     * Creates a new MatchDataLogger instance.
     * Initializes NetworkTables connection to Elastic/MatchData path.
     */
    public MatchDataLogger() {
        this.matchTable = NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("MatchData");
        this.matchStartTime = 0.0;
    }

    /**
     * Logs match start data including match info and timestamp.
     * Should be called at the beginning of autonomous or teleop.
     */
    public void logMatchStart() {
        matchStartTime = Timer.getFPGATimestamp();

        // Get match information from DriverStation
        int matchNumber = DriverStation.getMatchNumber();
        MatchType matchType = DriverStation.getMatchType();
        String eventName = DriverStation.getEventName();

        // Get alliance color
        String allianceColor = "Unknown";
        if (DriverStation.getAlliance().isPresent()) {
            Alliance alliance = DriverStation.getAlliance().get();
            allianceColor = (alliance == Alliance.Red) ? "Red" : "Blue";
        }

        // Publish to NetworkTables
        matchTable.getEntry("MatchNumber").setInteger(matchNumber);
        matchTable.getEntry("MatchType").setString(matchType.toString());
        matchTable.getEntry("EventName").setString(eventName);
        matchTable.getEntry("AllianceColor").setString(allianceColor);
        matchTable.getEntry("StartTime").setDouble(matchStartTime);
        matchTable.getEntry("MatchActive").setBoolean(true);
    }

    /**
     * Logs match end data and calculates total match duration.
     * Should be called when the match ends or robot is disabled.
     */
    public void logMatchEnd() {
        double matchEndTime = Timer.getFPGATimestamp();
        double matchDuration = matchEndTime - matchStartTime;

        matchTable.getEntry("EndTime").setDouble(matchEndTime);
        matchTable.getEntry("Duration").setDouble(matchDuration);
        matchTable.getEntry("MatchActive").setBoolean(false);
    }

    /**
     * Updates current match elapsed time.
     * Can be called periodically during the match to track progress.
     */
    public void updateElapsedTime() {
        if (matchStartTime > 0) {
            double elapsedTime = Timer.getFPGATimestamp() - matchStartTime;
            matchTable.getEntry("ElapsedTime").setDouble(elapsedTime);
        }
    }
}

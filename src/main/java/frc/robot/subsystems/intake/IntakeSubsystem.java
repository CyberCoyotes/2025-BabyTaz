/**
 * Represents the robot's intake mechanism as a subsystem.
 * This subsystem manages the control of the intake mechanism which is responsible for
 * picking up game pieces from the field.
 * 
 * This subsystem follows the command-based paradigm and implements hardware abstraction
 * through the IntakeIO interface. This allows for easy testing and hardware swapping.
 * 
 * Usage example:
 * <pre>
 * IntakeSubsystem intake = new IntakeSubsystem(new IntakeIOPhoenix6());
 * intake.setDefaultCommand(new IntakeCommand(intake));
 * </pre>
 * 
 * @see IntakeIO
 * @see IntakeIOPhoenix6
 */

 /*
  * Trying to modernize our code base we are going to use the new WPILib command-based
  */

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import com.playingwithfusion.TimeOfFlight;


// IntakeSubsystem.java - Now includes TOF sensor directly
public class IntakeSubsystem extends SubsystemBase {
    // Constants
    private static final int TOF_SENSOR_ID = 21;
    private static final double DEFAULT_INTAKE_SPEED = 0.25;
    private static final double DEFAULT_TOF_THRESHOLD_MM = 100.0;
    private static final double DEFAULT_CURRENT_LIMIT = 30.0;
    
    private final IntakeIO io;
    private final TimeOfFlight tofSensor;
    private final IntakeIOInputs inputs = new IntakeIOInputs();
    
    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
        this.tofSensor = new TimeOfFlight(TOF_SENSOR_ID);
        this.tofSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 24);
        
        initializeDashboard();
    }

    private void initializeDashboard() {
      SmartDashboard.putNumber("Intake/Speed", DEFAULT_INTAKE_SPEED);
      SmartDashboard.putNumber("Intake/TOF Threshold MM", DEFAULT_TOF_THRESHOLD_MM);
      SmartDashboard.putNumber("Intake/Current Limit", DEFAULT_CURRENT_LIMIT);
  }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Logger.processInputs("Intake", inputs);
        
        // Update dashboard
        SmartDashboard.putNumber("Intake/Current Draw", inputs.currentAmps);
        SmartDashboard.putNumber("Intake/Velocity", inputs.velocityRPM);
        SmartDashboard.putNumber("Intake/TOF Distance", tofSensor.getRange());
        SmartDashboard.putBoolean("Intake/Has Game Piece", hasGamePiece());
    }
    
 private double getIntakeSpeed() {
        return SmartDashboard.getNumber("Intake/Speed", DEFAULT_INTAKE_SPEED);
    }
    
    private double getTOFThresholdMM() {
        return SmartDashboard.getNumber("Intake/TOF Threshold MM", DEFAULT_TOF_THRESHOLD_MM);
    }
    
    private double getCurrentLimit() {
        return SmartDashboard.getNumber("Intake/Current Limit", DEFAULT_CURRENT_LIMIT);
    }
    
    // Public control methods
    public void runIntake() {
        io.setVoltage(getIntakeSpeed());
    }
    
    public void stopIntake() {
        io.stop();
    }
    
    public boolean hasGamePiece() {
        return inputs.tofDistanceMillimeters < getTOFThresholdMM();
    }
    
    // Command factories
    public Command runIntakeCommand() {
        return this.startEnd(
            this::runIntake,
            this::stopIntake
        ).until(this::hasGamePiece);
    }
    
    public Command stopIntakeCommand() {
        return this.runOnce(this::stopIntake);
    }
}

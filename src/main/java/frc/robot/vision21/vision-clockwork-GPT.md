# Clockwork Inspired from GPT

https://chatgpt.com/c/679ee2a2-21fc-800a-98e8-2707c800c798

Below is an example implementation that shows how you might set up AprilTag tracking using a Limelight 3 with NetworkTables plus a CTRE Phoenix 6 “Kraken‐based” drivetrain. This example is modeled on the style used by Team CLOCKWORK 2025: a dedicated vision constants file, a VisionSubsystem that “owns” all of the Limelight/vision processing, and a command (TagAlignCommand) that uses a PID loop to align the robot to an AprilTag target. (You can expand on this basic structure as needed.)

In this example the drivetrain is assumed to be a basic differential drive using CTRE Phoenix 6 motor controllers configured via the “Kraken” API. (In your actual robot code you may already have a more sophisticated drivetrain subsystem.) Be sure to adjust port numbers, PID gains, and tolerances to match your robot’s dynamics and testing results.

---

### VisionConstants.java

This file holds all of your vision-related tuning parameters. For example, it includes the PID proportional gain for alignment, a “deadband” (minimum command) to overcome static friction, a tolerance in degrees, and the name of the NetworkTables entry used by the Limelight.

```java
package frc.robot.Constants;

public final class VisionConstants {
    // PID tuning for horizontal alignment (tweak these values during testing)
    public static final double kAlignKp = 0.03;
    public static final double kAlignMinCommand = 0.05;
    public static final double kAlignTolerance = 1.0; // degrees

    // Limelight NetworkTables table name (default is "limelight")
    public static final String LIMELIGHT_TABLE = "limelight";
    
    // Pipeline number on the Limelight configured for AprilTag processing
    public static final int APRILTAG_PIPELINE = 1;
}
```

---

### VisionSubsystem.java

This subsystem is responsible for interacting with the Limelight. It reads values (like the horizontal offset “tx” and target validity “tv”) from NetworkTables, configures the appropriate pipeline for AprilTag tracking, and runs a simple PID loop to compute a turn command based on the error. You can also extend this class to compute target distance or pose (for example, using the known camera and target heights) if desired.

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final PIDController alignController;

    public VisionSubsystem() {
        // Get the Limelight NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_TABLE);

        // Configure the PID controller (only P used here, but you can add I and D if needed)
        alignController = new PIDController(VisionConstants.kAlignKp, 0, 0);
        alignController.setTolerance(VisionConstants.kAlignTolerance);

        // Set the Limelight to the AprilTag pipeline
        setPipeline(VisionConstants.APRILTAG_PIPELINE);
    }

    /** 
     * Sets the Limelight pipeline.
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Returns true if the Limelight has a valid target.
     */
    public boolean hasValidTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1.0;
    }

    /**
     * Gets the horizontal offset angle (in degrees) from the crosshair to the target.
     * (Typically ranges from -27 to 27 degrees.)
     */
    public double getTargetX() {
        return limelightTable.getEntry("tx").getDouble(0);
    }

    /**
     * (Optional) Gets the vertical offset angle from crosshair to target.
     */
    public double getTargetY() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * (Optional) Gets the target area (percentage of image area).
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /**
     * Uses the PID controller to calculate a turn command that will drive the horizontal error toward zero.
     */
    public double calculateTurn() {
        double error = getTargetX();
        double turnCommand = alignController.calculate(error, 0);
        
        // Apply a minimum command to overcome static friction if error is nonzero
        if (Math.abs(turnCommand) < VisionConstants.kAlignMinCommand && Math.abs(error) > VisionConstants.kAlignTolerance) {
            turnCommand = Math.copySign(VisionConstants.kAlignMinCommand, turnCommand);
        }
        return turnCommand;
    }
    
    @Override
    public void periodic() {
        // Optionally, publish vision data to the SmartDashboard for debugging:
        // SmartDashboard.putNumber("Vision/tx", getTargetX());
        // SmartDashboard.putBoolean("Vision/Valid", hasValidTarget());
    }
}
```

---

### TagAlignCommand.java

This command uses the VisionSubsystem to align the robot to an AprilTag. When executed, it checks for a valid target and then commands the drivetrain to rotate the robot until the horizontal offset error is within tolerance. (In this example, we are only turning in place. You might add forward/backward commands if you wish to also control the distance to the target.)

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.VisionConstants;

public class TagAlignCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final Drivetrain drivetrain;

    public TagAlignCommand(VisionSubsystem vision, Drivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        // If you are using a PID controller within VisionSubsystem, you may want to reset it here.
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            // Calculate a turn command based on horizontal error from the target.
            double turnCommand = vision.calculateTurn();
            // For this simple example, we drive with zero forward speed and apply the turn command.
            drivetrain.drive(0.0, turnCommand);
        } else {
            // No target found; you might choose to stop the robot or use a search routine.
            drivetrain.drive(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when a target is found and the error is within tolerance.
        return vision.hasValidTarget() && Math.abs(vision.getTargetX()) < VisionConstants.kAlignTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drivetrain.drive(0.0, 0.0);
    }
}
```

---

### Drivetrain.java (CTRE Phoenix 6 Kraken Example)

Below is a minimal drivetrain example that uses CTRE Phoenix 6 motor controllers. In this “Kraken‐based” example we assume a differential drive configuration where a simple combination of forward speed and rotation is used. Adjust the motor IDs, configuration settings, and drive calculations as needed for your actual robot.

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Import CTRE Phoenix 6 classes – adjust these imports if your project structure differs.
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.config.TalonFXConfiguration;
import com.ctre.phoenix6.ControlMode;

public class Drivetrain extends SubsystemBase {
    // Instantiate motor controllers (use your actual CAN IDs)
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public Drivetrain() {
        leftMotor = new TalonFX(1);
        rightMotor = new TalonFX(2);

        // Apply basic configuration to both motor controllers.
        TalonFXConfiguration config = new TalonFXConfiguration();
        // (Set any configuration parameters here as needed)
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);
    }

    /**
     * Drives the robot with a given forward speed and rotation command.
     *
     * @param speed    Forward speed command (from -1.0 to 1.0).
     * @param rotation Rotation command (from -1.0 to 1.0); positive values turn right.
     */
    public void drive(double speed, double rotation) {
        // For a differential drive:
        double leftOutput = speed + rotation;
        double rightOutput = speed - rotation;
        leftMotor.set(ControlMode.PercentOutput, leftOutput);
        rightMotor.set(ControlMode.PercentOutput, rightOutput);
    }
}
```

---

### Putting It Together

In your robot’s main command scheduler (for example, in your RobotContainer), you can instantiate the subsystems and bind the command (perhaps as a button command for vision alignment):

```java
// In RobotContainer.java (or your equivalent command-based setup class):

// Instantiate subsystems
private final VisionSubsystem visionSubsystem = new VisionSubsystem();
private final Drivetrain drivetrain = new Drivetrain();

// Create the command
private final TagAlignCommand tagAlignCommand = new TagAlignCommand(visionSubsystem, drivetrain);

// For example, bind the command to a joystick button:
// new JoystickButton(joystick, buttonNumber).whenHeld(tagAlignCommand);
```

This setup mirrors the structured approach used by Team CLOCKWORK 2025: vision is encapsulated in its own subsystem with a dedicated constants file and a command that leverages vision feedback to control the drivetrain. You can expand this framework by adding additional processing (such as calculating distance to the AprilTag using known target dimensions) or by incorporating multi-axis alignment as needed.

Adjust and test all PID gains, command parameters, and hardware configurations during your practice sessions so that the system behaves reliably on your robot.
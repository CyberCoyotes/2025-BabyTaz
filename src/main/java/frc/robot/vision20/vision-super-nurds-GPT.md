# Super Nurds GPT version

https://chatgpt.com/g/g-p-679edf5bce708191a1c88dfcea2678d3-frc-3603-coding/c/679ee1d0-1c98-800a-b0de-e00fd1da8631

Below is an example implementation modeled after the SUPER_NURDS 2025 vision approach. In this example the vision code uses the Limelight’s network table entries to retrieve apriltag information and then drives the robot using a CTRE Phoenix 6 Kraken–based drivetrain. (It’s assumed you have a drivetrain subsystem (here referenced as DrivetrainSubsystem) that wraps your Kraken hardware.) You can adjust constants and thresholds to suit your robot’s geometry and tuning.

---

### VisionConstants.java

This constants class holds key parameters for your Limelight mounting, target dimensions, and PID tuning for tag alignment.

```java
package frc.robot.Constants;

public final class VisionConstants {
    // Limelight physical mounting and target configuration (in meters and degrees)
    public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 30.0; // Adjust to your limelight angle
    public static final double LIMELIGHT_HEIGHT_METERS = 0.5;           // Height of your Limelight off the floor
    public static final double TARGET_HEIGHT_METERS = 2.5;              // Height of the apriltag target

    // PID gains for tag alignment (tune these for your robot)
    public static final double ALIGN_P = 0.05;
    public static final double ALIGN_I = 0.0;
    public static final double ALIGN_D = 0.0;

    // Desired horizontal offset from target (in degrees) when aligned
    public static final double ALIGN_TARGET_OFFSET_DEGREES = 0.0;

    // Acceptable error threshold (deadband) for alignment
    public static final double ALIGN_DEADBAND_DEGREES = 1.0;

    // Maximum speeds (tweak these to match your drivetrain capabilities)
    public static final double MAX_ROTATION_SPEED = 0.5; // Maximum rotational speed
    public static final double MAX_FORWARD_SPEED = 0.3;  // Forward speed when approaching the target

    // Example threshold for distance-based adjustments (in meters)
    public static final double APPROACH_DISTANCE_THRESHOLD = 1.0;
}
```

---

### VisionSubsystem.java

This subsystem handles all interactions with the Limelight. It reads network table entries (like "tx", "ty", "ta") and computes useful values such as distance based on the mounting angle and vertical offset. It also provides methods to change pipelines and LED modes.

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    // Access the Limelight's network table (default name is "limelight")
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Returns true if the Limelight has detected a valid target.
     */
    public boolean hasValidTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    /**
     * Returns the horizontal offset from the crosshair to the target (in degrees).
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical offset from the crosshair to the target (in degrees).
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Returns the target area (useful for estimating target distance).
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * If your pipeline outputs an apriltag ID (for example, via a "tid" key), retrieve it.
     * Adjust the key if necessary.
     */
    public int getAprilTagID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Estimate the distance to the target using the limelight’s mounting angle and the vertical offset.
     * Formula: distance = (targetHeight - limelightHeight) / tan(mountingAngle + ty)
     */
    public double calculateDistance() {
        double angleToTargetRadians = Math.toRadians(VisionConstants.LIMELIGHT_MOUNTING_ANGLE_DEGREES + getVerticalOffset());
        return (VisionConstants.TARGET_HEIGHT_METERS - VisionConstants.LIMELIGHT_HEIGHT_METERS) / Math.tan(angleToTargetRadians);
    }

    /**
     * Set the limelight's pipeline.
     * @param pipeline The pipeline index to set.
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Set the LED mode for the limelight.
     * @param mode LED mode value (e.g., 3 for force on, 1 for default).
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    @Override
    public void periodic() {
        // Optional: post vision data to SmartDashboard or log telemetry here
    }
}
```

---

### TagAlignCommand.java

This command uses a PID loop (built with WPILib’s PIDController) to rotate the robot until the horizontal offset (tx) is near zero. Optionally, it commands a small forward motion if the robot is far from the target. It assumes you have a drivetrain subsystem (using CTRE Phoenix 6 Kraken controllers) with a method like `drive(forward, rotation)`.

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.controller.PIDController;

public class TagAlignCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final PIDController rotationController;

    public TagAlignCommand(VisionSubsystem vision, DrivetrainSubsystem drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        addRequirements(vision, drivetrain);

        // Initialize the PID controller with constants defined in VisionConstants.
        rotationController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
        rotationController.setSetpoint(VisionConstants.ALIGN_TARGET_OFFSET_DEGREES);
        rotationController.setTolerance(VisionConstants.ALIGN_DEADBAND_DEGREES);
    }

    @Override
    public void initialize() {
        // Set the limelight pipeline to the one configured for apriltag detection.
        vision.setPipeline(1); // Ensure pipeline 1 is configured for apriltags
        // Force the limelight LEDs on for reliable vision
        vision.setLEDMode(3);
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            // Get horizontal offset error from target.
            double tx = vision.getHorizontalOffset();

            // Use the PID controller to compute a rotational speed.
            double rotationSpeed = rotationController.calculate(tx);

            // Clamp the rotation speed to the maximum allowed value.
            rotationSpeed = Math.max(-VisionConstants.MAX_ROTATION_SPEED, Math.min(VisionConstants.MAX_ROTATION_SPEED, rotationSpeed));

            // Optionally, compute a forward speed based on distance (here, drive forward if too far).
            double distance = vision.calculateDistance();
            double forwardSpeed = 0.0;
            if (distance > VisionConstants.APPROACH_DISTANCE_THRESHOLD) {
                forwardSpeed = VisionConstants.MAX_FORWARD_SPEED;
            }

            // Command the drivetrain.
            drivetrain.drive(forwardSpeed, rotationSpeed);
        } else {
            // If no target is visible, stop the drivetrain.
            drivetrain.drive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        // End command when the target is valid and we are within the deadband.
        return vision.hasValidTarget() && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drivetrain.drive(0, 0);
        // Optionally, reset the limelight to its default LED mode or pipeline.
        vision.setLEDMode(1);
    }
}
```

---

### Additional Notes

- **DrivetrainSubsystem:**  
  This example assumes you have a drivetrain subsystem that wraps your CTRE Phoenix 6 Kraken motor controllers. In your drivetrain code, implement a method such as:
  ```java
  public void drive(double forward, double rotation) {
      // Example: Convert forward and rotation values to motor outputs.
      // Use CTRE Phoenix 6 Kraken API to drive your motors.
  }
  ```
  Adjust the drive method to account for your specific hardware configuration.

- **Tuning:**  
  The PID constants, speed limits, and distance thresholds are starting points. You’ll need to tune these based on your robot’s response and target distances.

- **Pipeline and Network Table Keys:**  
  Ensure that the Limelight is configured to use the correct pipeline (here assumed to be pipeline 1 for apriltags) and that your network table keys (such as `"tx"`, `"ty"`, `"ta"`, and `"tv"`) match those used by your Limelight firmware. If your apriltag detection outputs additional information (for example, a tag ID under a different key), adjust the code accordingly.

This sample code should provide a strong foundation for integrating apriltag tracking with a Limelight 3 and a CTRE Phoenix 6 Kraken–based drivetrain, modeled after the SUPER_NURDS 2025 approach. Adjust and expand the code as needed for your team’s specific hardware and strategies.
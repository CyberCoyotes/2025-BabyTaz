# Generic Vision Model GPT

https://chatgpt.com/c/679ee334-9e1c-800a-b1e5-4cf7c8f689ff

Not based on any specific team's model

Below is an example implementation in Java using WPILib’s command‐based framework. This example assumes you already have a drivetrain subsystem built on CTRE Phoenix 6 Kraken controllers (with an arcadeDrive or similar method) and that you’re running a Limelight 3 set up with an AprilTag pipeline. The code is modeled after the Team Mechanical Advantage (TMA) vision approach, with a dedicated vision subsystem that reads network table entries from the Limelight and a command that uses a PID loop to align the robot to an AprilTag target.

---

### VisionSubsystem

This subsystem accesses the Limelight’s network table entries. In addition to the basic target data (horizontal offset, vertical offset, area, and skew), it includes helper methods to change the LED and pipeline mode. (You can expand this with distance estimation using camera and target height if desired.)

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VisionSubsystem encapsulates all vision-related functionality.
 * It interfaces with the Limelight to retrieve target data.
 */
public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public VisionSubsystem() {
        // Connect to the Limelight's NetworkTable
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Determines if the Limelight has a valid target.
     * @return true if a target is detected.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Gets the horizontal offset from the crosshair to the target.
     * @return the horizontal error in degrees.
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Gets the vertical offset from the crosshair to the target.
     * @return the vertical error in degrees.
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the target area (as a percentage of the image).
     * @return the target area.
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0.0);
    }

    /**
     * Gets the skew or rotation of the target.
     * @return the target skew in degrees.
     */
    public double getSkew() {
        return limelightTable.getEntry("ts").getDouble(0.0);
    }

    /**
     * Sets the LED mode of the Limelight.
     * Modes may vary (for example, 0 for pipeline default, 1 for force off, 3 for force on).
     * @param mode the LED mode.
     */
    public void setLEDMode(int mode) {
        limelightTable.getEntry("ledMode").setNumber(mode);
    }

    /**
     * Sets the pipeline on the Limelight.
     * @param pipeline the pipeline number.
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
}
```

---

### VisionConstants

A constants class holds key PID tuning values and any offsets you might need (for example, to account for a camera not being mounted perfectly aligned). Adjust these values to suit your robot’s performance.

```java
package frc.robot.Constants;

/**
 * Constants used by the vision system.
 */
public final class VisionConstants {
    // PID constants for vision-based alignment (tune these values)
    public static final double kVisionAlignKP = 0.02;
    public static final double kVisionAlignKI = 0.0;
    public static final double kVisionAlignKD = 0.0;
    
    // Acceptable tolerance (in degrees) for considering the robot aligned.
    public static final double kVisionAlignTolerance = 1.0;
    
    // If your camera is not mounted exactly in line with the robot's center,
    // adjust the target offset (in degrees) here.
    public static final double kTargetOffset = 0.0;
    
    // Additional constants for distance estimation (if needed)
    public static final double kCameraHeight = 0.5;    // meters (example value)
    public static final double kTargetHeight = 2.5;    // meters (example value)
    public static final double kCameraAngle = 30.0;      // degrees (example value)
}
```

---

### TagAlignCommand

This command uses the vision subsystem’s target data and a PID controller to rotate the robot so that it is aligned with the AprilTag. It assumes your drivetrain subsystem (built on CTRE Phoenix 6 Kraken controllers) has an `arcadeDrive(double fwd, double rot)` method.

```java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * A command that aligns the robot to an AprilTag using vision data.
 * It reads the horizontal offset from the VisionSubsystem and applies
 * a PID controller to rotate the robot until it is aligned.
 */
public class TagAlignCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final DrivetrainSubsystem drivetrain;
    private final PIDController pidController;

    /**
     * Constructs a new TagAlignCommand.
     * @param vision The vision subsystem that provides target data.
     * @param drivetrain The drivetrain subsystem (using CTRE Phoenix 6 Kraken controllers).
     */
    public TagAlignCommand(VisionSubsystem vision, DrivetrainSubsystem drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        pidController = new PIDController(VisionConstants.kVisionAlignKP, VisionConstants.kVisionAlignKI, VisionConstants.kVisionAlignKD);
        pidController.setTolerance(VisionConstants.kVisionAlignTolerance);
        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        // Reset the PID controller and enable the Limelight LEDs.
        pidController.reset();
        vision.setLEDMode(3); // Force LEDs on (mode 3 typically forces on)
    }

    @Override
    public void execute() {
        if (vision.hasTarget()) {
            // Calculate the error between the current horizontal offset and the desired offset.
            double error = vision.getHorizontalOffset() - VisionConstants.kTargetOffset;
            // Compute the PID output to determine rotation speed.
            double rotationSpeed = pidController.calculate(error, 0.0);
            
            // Command the drivetrain: zero forward speed, rotation based on PID output.
            drivetrain.arcadeDrive(0.0, rotationSpeed);
        } else {
            // If no target is detected, stop any rotational movement.
            drivetrain.arcadeDrive(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        // End the command when a target is present and the alignment error is within tolerance.
        return vision.hasTarget() && pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop drivetrain motion when the command ends.
        drivetrain.arcadeDrive(0.0, 0.0);
        // Optionally, revert the LED mode to your default setting.
        vision.setLEDMode(1); // Example: mode 1 might turn the LED off or use pipeline default
    }
}
```

---

### Integration Notes

- **DrivetrainSubsystem:**  
  This example assumes you have a drivetrain subsystem (e.g., `DrivetrainSubsystem.java`) that wraps your CTRE Phoenix 6 Kraken hardware and provides an `arcadeDrive(double, double)` method. Make sure that subsystem is correctly integrated into your robot’s command scheduler.

- **Tuning:**  
  The PID constants in `VisionConstants` are starting values. You’ll likely need to tune these gains based on your robot’s dynamics and the Limelight’s measurements.

- **Limelight Configuration:**  
  Ensure that your Limelight 3 is configured with an AprilTag detection pipeline. You may also wish to include functions in your vision subsystem to calculate the distance to the target using trigonometric methods if your application requires it.

- **Team Mechanical Advantage Approach:**  
  This design mirrors the TMA approach by centralizing vision processing in a dedicated subsystem and then “consuming” that data in a command that makes real-time corrections. It keeps your vision code modular and separate from your drivetrain logic.

This sample should serve as a strong starting point for implementing AprilTag tracking on your FRC robot using Limelight 3 and CTRE Phoenix 6 Kraken-based hardware.
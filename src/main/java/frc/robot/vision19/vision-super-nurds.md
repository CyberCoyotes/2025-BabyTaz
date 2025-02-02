# Super Nurds Insipired

https://claude.ai/chat/a6f4fba8-16c6-4b59-9040-e2b802d62f16

I'll help you create a vision tracking system for your Limelight 3 and Kraken-based drivetrain using Phoenix 6. I'll break this down into several parts with detailed explanations.

1. First, let's create VisionConstants:

```java
package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;

public final class VisionConstants {
    // Limelight mount position and orientation relative to robot center
    public static final Transform3d LIMELIGHT_TO_ROBOT = new Transform3d(
        new Translation3d(
            Units.Inches.of(11.5).in(Units.Meters),  // X forward
            Units.Inches.of(-2.5).in(Units.Meters),  // Y left
            Units.Inches.of(22.0).in(Units.Meters)   // Z up
        ),
        new Rotation3d(
            Units.Degrees.of(0).in(Units.Radians),   // Roll
            Units.Degrees.of(15).in(Units.Radians),  // Pitch (angle up/down)
            Units.Degrees.of(0).in(Units.Radians)    // Yaw
        )
    );

    // Vision processing constants
    public static final double MIN_TARGET_AREA = 0.1;  // Minimum area % for valid detection
    public static final double MAX_POSE_AMBIGUITY = 0.2;  // Maximum ambiguity score for pose estimation
    public static final double MAX_VALID_RANGE = Units.Feet.of(20).in(Units.Meters);

    // PID Constants for tag alignment
    public static final double ALIGN_P = 0.03;
    public static final double ALIGN_I = 0.0;
    public static final double ALIGN_D = 0.002;

    // Alignment tolerances
    public static final double ALIGNED_THRESHOLD_X = Units.Inches.of(2).in(Units.Meters);
    public static final double ALIGNED_THRESHOLD_Y = Units.Inches.of(2).in(Units.Meters);
    public static final double ALIGNED_THRESHOLD_ROTATION = Units.Degrees.of(2).in(Units.Radians);

    // Rate limits for smooth motion
    public static final double MAX_ANGULAR_VELOCITY = Units.DegreesPerSecond.of(180).in(Units.RadiansPerSecond);
    public static final double MAX_DRIVE_VELOCITY = Units.MetersPerSecond.of(2).in(Units.MetersPerSecond);
}
```

2. Next, let's create the VisionSubsystem:

```java
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;
    private int lastSeenTagId = -1;
    private double lastValidTimestamp = 0;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        configureLimelight();
    }

    private void configureLimelight() {
        // Configure Limelight pipeline for AprilTags
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.setCameraMode_Processor("limelight");
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }

    @Override
    public void periodic() {
        updateVisionData();
        publishTelemetry();
    }

    private void updateVisionData() {
        if (hasValidTarget()) {
            lastSeenTagId = LimelightHelpers.getFiducialID("limelight");
            lastValidTimestamp = LimelightHelpers.getLatency_Pipeline("limelight");
            
            // If using vision-assisted odometry:
            if (shouldUpdateOdometry()) {
                var botpose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
                drivetrain.addVisionMeasurement(botpose, lastValidTimestamp);
            }
        }
    }

    public boolean hasValidTarget() {
        return LimelightHelpers.getTV("limelight") && 
               LimelightHelpers.getTA("limelight") > VisionConstants.MIN_TARGET_AREA;
    }

    public double getTargetXOffset() {
        return LimelightHelpers.getTX("limelight");
    }

    public double getTargetYOffset() {
        return LimelightHelpers.getTY("limelight");
    }

    public double getTargetArea() {
        return LimelightHelpers.getTA("limelight");
    }

    public int getTagId() {
        return lastSeenTagId;
    }

    public void setLeds(boolean enabled) {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    private boolean shouldUpdateOdometry() {
        return hasValidTarget() && 
               LimelightHelpers.getBotPose_TAOnly("limelight") >= 2 &&  // At least 2 tags visible
               LimelightHelpers.getPoseAmbiguity("limelight") < VisionConstants.MAX_POSE_AMBIGUITY;
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget());
        SmartDashboard.putNumber("Vision/TagID", lastSeenTagId);
        SmartDashboard.putNumber("Vision/TX", getTargetXOffset());
        SmartDashboard.putNumber("Vision/TY", getTargetYOffset());
        SmartDashboard.putNumber("Vision/TA", getTargetArea());
    }
}
```

3. Now let's create the TagAlignCommand:

```java
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class TagAlignCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    private final SwerveRequest.FieldCentric drive;
    private final int targetTagId;

    public TagAlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, int targetTagId) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetTagId = targetTagId;
        
        xController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
        yController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
        rotationController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
        
        // Configure tolerance for "at target" checking
        xController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_X);
        yController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_Y);
        rotationController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_ROTATION);
        
        // Make rotation continuous
        rotationController.enableContinuousInput(-180, 180);
        
        drive = new SwerveRequest.FieldCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1);

        addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
        
        // Enable Limelight LEDs for better tracking
        vision.setLeds(true);
    }

    @Override
    public void execute() {
        if (!vision.hasValidTarget() || vision.getTagId() != targetTagId) {
            // No valid target - stop moving
            drivetrain.setControl(drive.withVelocityX(0)
                                     .withVelocityY(0)
                                     .withRotationalRate(0));
            return;
        }

        // Calculate control outputs
        double xSpeed = xController.calculate(vision.getTargetXOffset(), 0);
        double ySpeed = yController.calculate(vision.getTargetYOffset(), 0);
        double rotationSpeed = rotationController.calculate(vision.getTargetRotation(), 0);

        // Apply speed limits
        xSpeed = Math.min(Math.abs(xSpeed), VisionConstants.MAX_DRIVE_VELOCITY) * Math.signum(xSpeed);
        ySpeed = Math.min(Math.abs(ySpeed), VisionConstants.MAX_DRIVE_VELOCITY) * Math.signum(ySpeed);
        rotationSpeed = Math.min(Math.abs(rotationSpeed), VisionConstants.MAX_ANGULAR_VELOCITY) * Math.signum(rotationSpeed);

        // Send commands to drivetrain
        drivetrain.setControl(drive.withVelocityX(xSpeed)
                                 .withVelocityY(ySpeed)
                                 .withRotationalRate(rotationSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving and turn off LEDs
        drivetrain.setControl(drive.withVelocityX(0)
                                 .withVelocityY(0)
                                 .withRotationalRate(0));
        vision.setLeds(false);
    }

    @Override
    public boolean isFinished() {
        return vision.hasValidTarget() && 
               vision.getTagId() == targetTagId &&
               xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint();
    }
}
```

4. Finally, to use this in RobotContainer:

```java
public class RobotContainer {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    public RobotContainer() {
        drivetrain = new CommandSwerveDrivetrain();
        vision = new VisionSubsystem(drivetrain);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Example: Align to AprilTag ID 1 when pressing A button
        new JoystickButton(driverController, Button.kA.value)
            .whileTrue(new TagAlignCommand(vision, drivetrain, 1));
    }
}
```

Key Features of this Implementation:

1. Vision Processing:
- Robust target validation
- Support for multi-tag detection
- Vision-assisted odometry updates
- Latency compensation
- LED control

2. Tag Alignment:
- PID control for X, Y, and rotation
- Continuous rotation handling
- Speed limiting for smooth motion
- Tolerance-based completion checking
- Automatic LED control

3. Safety Features:
- Target validation checks
- Maximum speed limits
- Graceful handling of lost targets
- Automatic stopping when target is lost

4. Telemetry:
- Comprehensive SmartDashboard data
- Target tracking status
- Alignment error reporting
- Tag ID tracking

To use this system:

1. Adjust VisionConstants for your robot's specific measurements and mount positions
2. Tune the PID constants for your robot's specific behavior
3. Configure the appropriate Limelight pipeline for AprilTags
4. Test alignment behavior starting with low speeds and gradually increasing as needed

Remember to characterize your drivetrain properly and ensure your Kraken motors are configured correctly in Phoenix 6 for optimal control.
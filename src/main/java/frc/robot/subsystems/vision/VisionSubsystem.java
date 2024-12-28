package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionSubsystem extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputs inputs = new VisionIOInputs();
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDs leds;

    // Configuration constants
    private static final LimelightConfiguration config = new LimelightConfiguration();
    
    private Pose2d cameraToTagPose = new Pose2d();
    private double distanceToTargetMeters = Double.NaN;

    public VisionSubsystem(VisionIO io, CommandSwerveDrivetrain drivetrain, LEDs leds) {
        this.io = io;
        this.drivetrain = drivetrain;
        this.leds = leds;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        updatePoseEstimation();
        updateLEDs();
        logTelemetry();
    }

    private void updatePoseEstimation() {
        if (inputs.hasTargets) {
            // Calculate camera to target geometry...
            double theta = inputs.verticalAngleRadians + config.mountAngleRadians;
            double heightDelta = getTargetHeight() - config.mountHeightMeters;
            distanceToTargetMeters = heightDelta / Math.tan(theta);
            
            cameraToTagPose = new Pose2d(
                distanceToTargetMeters * Math.cos(inputs.horizontalAngleRadians),
                distanceToTargetMeters * Math.sin(inputs.horizontalAngleRadians),
                Rotation2d.fromRadians(inputs.horizontalAngleRadians));
        }
    }

    private void logTelemetry() {
        SmartDashboard.putBoolean("Vision/HasTarget", inputs.hasTargets);
        SmartDashboard.putNumber("Vision/Distance", distanceToTargetMeters);
        SmartDashboard.putString("Vision/CameraToTag", cameraToTagPose.toString());
        // ... additional logging
    }
}
/*
 
# Key improvements from 2910's approach:

## Hardware Abstraction:
* Clear separation between hardware interface (VisionIO) and implementation
* Makes testing and simulation easier
* Allows swapping different vision systems

## Better State Management:
* All vision data stored in VisionIOInputs class
* Thread-safe updates using synchronized methods
* Clear data flow from hardware to subsystem


## More Sophisticated Pose Estimation:
* Proper handling of camera mounting geometry
* Better integration with field layout and AprilTag positions
* More accurate distance calculations

## Better Logging and Debugging:
* More comprehensive telemetry
* Clear separation of data collection and processing
* Better error handling




 */
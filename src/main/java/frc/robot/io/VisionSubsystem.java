package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.LimelightHelpers;
import frc.robot.io.VisionState;
import frc.robot.io.led.LEDIO;
import frc.robot.io.led.LEDs;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDs ledSubsystem;
    
    private VisionState currentState = VisionState.NO_TARGET;
    private double lastValidTargetTimestamp = 0;

    // Constants
    private static final double ALIGNMENT_TOLERANCE_X = 1.0; // degrees
    private static final double ALIGNMENT_TOLERANCE_Y = 1.0; // degrees
    private static final double ALIGNMENT_TOLERANCE_AREA = 0.1;
    private static final double TARGET_LOSS_TIMEOUT = 0.5; // seconds

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, LEDIO leds) {
        this.drivetrain = drivetrain;
        this.ledSubsystem = leds;
    }

    @Override
    public void periodic() {
        updateVisionState();
        updateLEDs();
        updateSmartDashboard();
    }

    private void updateVisionState() {
        if (!hasValidTarget()) {
            currentState = VisionState.NO_TARGET;
            return;
        }

        lastValidTargetTimestamp = Timer.getFPGATimestamp();
        
        // Check if we're aligned with the target
        boolean isAligned = Math.abs(getTargetXOffset()) < ALIGNMENT_TOLERANCE_X &&
                          Math.abs(getTargetYOffset()) < ALIGNMENT_TOLERANCE_Y;

        currentState = isAligned ? VisionState.TARGET_ALIGNED : VisionState.TARGET_DETECTED;
    }

    private void updateLEDs() {
        switch (currentState) {
            case NO_TARGET:
                ledSubsystem.setState(LEDs.LEDState.NO_TARGET); // Red
                break;
            case TARGET_DETECTED:
                ledSubsystem.setState(LEDs.LEDState.TARGET_VISIBLE); // Yellow
                break;
            case TARGET_ALIGNED:
                ledSubsystem.setState(LEDs.LEDState.TARGET_LOCKED); // Green
                break;
        }
    }

    private void updateSmartDashboard() {
        // Vision Status
        SmartDashboard.putString("Vision/State", currentState.getDescription());
        SmartDashboard.putBoolean("Vision/HasTarget", hasValidTarget());
        
        // Target Information
        SmartDashboard.putNumber("Vision/TX", getTargetXOffset());
        SmartDashboard.putNumber("Vision/TY", getTargetYOffset());
        SmartDashboard.putNumber("Vision/TA", getTargetArea());
        
        // AprilTag Data
        if (hasValidTarget()) {
            PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (poseEstimate != null) {
                SmartDashboard.putNumber("Vision/TagCount", poseEstimate.tagCount);
                SmartDashboard.putNumber("Vision/TagDistance", poseEstimate.avgTagDist);
                SmartDashboard.putNumber("Vision/TagArea", poseEstimate.avgTagArea);
                SmartDashboard.putNumber("Vision/Latency", poseEstimate.latency);
                
                // Robot pose data
                Pose2d robotPose = poseEstimate.pose;
                SmartDashboard.putNumber("Vision/RobotX", robotPose.getX());
                SmartDashboard.putNumber("Vision/RobotY", robotPose.getY());
                SmartDashboard.putNumber("Vision/RobotRotation", robotPose.getRotation().getDegrees());
            }
        }

        // Alignment Status
        SmartDashboard.putBoolean("Vision/IsAligned", currentState == VisionState.TARGET_ALIGNED);
    }

    // Getters for vision state
    public VisionState getCurrentState() {
        return currentState;
    }

    public boolean isAligned() {
        return currentState == VisionState.TARGET_ALIGNED;
    }
}
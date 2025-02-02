package frc.robot.vision21;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


import frc.robot.vision21.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final PIDController alignController;
    private final PhoenixPIDController alignPhoenixController;

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
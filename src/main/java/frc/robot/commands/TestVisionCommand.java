/**
 * File: TestVisionAlignCommand.java
 * Description: This file contains the TestVisionAlignCommand class, which is a command that uses the VisionSubsystem to align the robot to a target. 
 * This command is used for testing purposes and is not intended for use in competition.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.Color;

public class TestVisionCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final LEDSubsystem leds;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    // Previous constants remain the same...
    private static final double ROTATION_KP = 0.03;
    private static final double MAX_ROTATION_SPEED = 0.2;

    // Add alignment threshold
    private static final double ALIGNED_THRESHOLD = 2.0; // Degrees

    public TestVisionCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, LEDSubsystem leds) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.leds = leds;
        addRequirements(vision, drivetrain, leds);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Vision Test/Running", true);
        leds.setColor(Color.GOLD);
    }

    @Override
    public void execute() {
        if (vision.hasValidTarget()) {
            double xError = vision.getTargetXAngle();
            int tagId = vision.getCurrentTagID();

/* 
            // Calculate rotation speed
            double rotationSpeed = -xError * ROTATION_KP;
            rotationSpeed = Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationSpeed));
*/
            // Update LEDs based on alignment
            if (tagId == 1 || tagId == 2 || tagId == 3 || tagId == 4) {
                leds.setColor(Color.GREEN);
            } else {
                leds.setColor(Color.PURPLE);
            }
/*
            // Apply the rotation directly
            drive.withRotationalRate(rotationSpeed);
            drivetrain.applyRequest(() -> drive);
*/
            // Output debug data
            SmartDashboard.putNumber("Vision Test/Tag ID", tagId);
            SmartDashboard.putNumber("Vision Test/X Error", xError);
            
        // } else {
            // leds.setRed();
            // drive.withRotationalRate(0);
            // drivetrain.applyRequest(() -> drive);

            // SmartDashboard.putNumber("Vision Test/Tag ID", -1);
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving and turn off LEDs
        drivetrain.applyRequest(() -> drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        leds.setOff();
        SmartDashboard.putBoolean("Vision Test/Running", false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
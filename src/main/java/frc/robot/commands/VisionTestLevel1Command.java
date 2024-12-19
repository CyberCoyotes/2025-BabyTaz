// Create new file: src/main/java/frc/robot/commands/VisionTestLevel1Command.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.VisionState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class VisionTestLevel1Command extends Command {
    private final VisionSubsystem vision;
    private final LEDSubsystem leds;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    
    private static final double TEST_SPEED = 0.1; // 10% speed
    private static final double TEST_DURATION = 10.0; // 10 seconds
    private double startTime;

    public VisionTestLevel1Command(VisionSubsystem vision, LEDSubsystem leds, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.leds = leds;
        this.drivetrain = drivetrain;
        addRequirements(vision, leds, drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        VisionState state = vision.getCurrentState();
        
        // Update LEDs based on state
        switch(state) {
            case TARGET_1:
                leds.setColor(LEDSubsystem.Color.RED);
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(-TEST_SPEED) // Left
                    .withRotationalRate(0));
                break;
            case TARGET_2:
                leds.setColor(LEDSubsystem.Color.GREEN);
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(TEST_SPEED) // Right
                    .withRotationalRate(0));
                break;
            case TARGET_3:
                leds.setColor(LEDSubsystem.Color.BLUE);
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(TEST_SPEED) // Forward
                    .withVelocityY(0)
                    .withRotationalRate(0));
                break;
            case TARGET_4:
                leds.setColor(LEDSubsystem.Color.YELLOW);
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(-TEST_SPEED) // Backward
                    .withVelocityY(0)
                    .withRotationalRate(0));
                break;
            default:
                leds.setColor(LEDSubsystem.Color.PURPLE);
                drivetrain.applyRequest(() -> drive
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(0));
                break;
        }
    }

    @Override 
    public void end(boolean interrupted) {
        // Stop movement and set LEDs off
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
        leds.setOff();
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() / 1000.0 - startTime) > TEST_DURATION;
    }
}
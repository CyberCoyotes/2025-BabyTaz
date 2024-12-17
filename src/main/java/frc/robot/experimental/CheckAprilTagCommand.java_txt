// CheckAprilTagCommand.java
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.RobotState;

public class CheckAprilTagCommand extends Command {
    private final VisionSubsystem visionSubsystem;
    private final LEDSubsystem ledSubsystem;

    // Define colors for different tags (RGB values)
    private static final int[] BLUE = {0, 0, 255};      // Tag 1 - Loading
    private static final int[] PURPLE = {148, 0, 211};  // Tag 2 - Left Score
    private static final int[] YELLOW = {255, 255, 0};  // Tag 3 - Center Score
    private static final int[] GREEN = {0, 255, 0};     // Tag 4 - Right Score
    private static final int[] RED = {255, 0, 0};       // No target/default

    public CheckAprilTagCommand(VisionSubsystem visionSubsystem, CommandSwerveDrivetrain drivetrain) {
        this.visionSubsystem = visionSubsystem;
        this.ledSubsystem = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset LEDs to default state when command starts
        ledSubsystem.setColor(RED);
    }

    @Override
    public void execute() {
        RobotState currentState;
        int[] ledColor;

        if (visionSubsystem.hasValidTarget()) {
            int tagId = visionSubsystem.getCurrentTagID();
            
            // Determine state and color based on tag ID
            switch (tagId) {
                case 1:
                    currentState = RobotState.TARGETING_TAG_1;
                    ledColor = BLUE;
                    break;
                case 2:
                    currentState = RobotState.TARGETING_TAG_2;
                    ledColor = PURPLE;
                    break;
                case 3:
                    currentState = RobotState.TARGETING_TAG_3;
                    ledColor = YELLOW;
                    break;
                case 4:
                    currentState = RobotState.TARGETING_TAG_4;
                    ledColor = GREEN;
                    break;
                default:
                    currentState = RobotState.NO_TARGET;
                    ledColor = RED;
                    break;
            }
        } else {
            currentState = RobotState.NO_TARGET;
            ledColor = RED;
        }

        // Update SmartDashboard
        SmartDashboard.putString("Robot State", currentState.name());
        SmartDashboard.putString("State Description", currentState.getDescription());
        
        // Update LEDs
        ledSubsystem.setColor(ledColor);
    }

    @Override
    public void end(boolean interrupted) {
        // Set LEDs back to default state when command ends
        ledSubsystem.setColor(RED);
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
}
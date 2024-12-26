// Create new file: src/main/java/frc/robot/commands/VisionTestLevel2Command.java
package frc.robot.experimental;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.VisionState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class VisionTestLevel2Command extends Command {
    private final VisionSubsystem vision;
    private final LEDSubsystem leds;
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    
    // Distance in meters (0.20 = 20cm)
    private static final double TEST_SPEED = 0.1; // 10% speed, same as Level 1
    private static final double MAX_ROTATION_SPEED = 0.1; // 10% rotation speed

    private static final double TARGET_DISTANCE = 0.20; // How far from tag we want to be
    private static final double DISTANCE_TOLERANCE = 0.02; // +/- 2cm tolerance
    
    public VisionTestLevel2Command(VisionSubsystem vision, LEDSubsystem leds, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.leds = leds;
        this.drivetrain = drivetrain;
        addRequirements(vision, leds, drivetrain);
    }

    @Override
    public void execute() {
        if (!vision.hasValidTarget()) {
            handleNoTarget();
            return;
        }

        int tagId = vision.getCurrentTagID();
        double currentDistance = vision.getTargetZDistance();
        boolean isInRange = Math.abs(currentDistance - TARGET_DISTANCE) < DISTANCE_TOLERANCE;
        
        // Update dashboard with current status
        SmartDashboard.putNumber("Target Distance (m)", currentDistance);
        SmartDashboard.putBoolean("In Range", isInRange);
        SmartDashboard.putNumber("Tag ID", tagId);

        if (isInRange) {
            handleTargetInRange(tagId);
        } else {
            handleTargetOutOfRange(tagId, currentDistance);
        }
    }

    private void handleNoTarget() {
        leds.setColor(LEDSubsystem.Color.PURPLE);
        stopMovement();
    }

    private void handleTargetInRange(int tagId) {
        // Set different colors for in-range targets
        switch(tagId) {
            case 1:
                leds.setColor(LEDSubsystem.Color.GREEN); // Bright green when in range
                break;
            case 2:
                leds.setColor(LEDSubsystem.Color.CYAN);
                break;
            case 3:
                leds.setColor(LEDSubsystem.Color.GOLD);
                break;
            case 4:
                leds.setColor(LEDSubsystem.Color.WHITE);
                break;
            default:
                leds.setColor(LEDSubsystem.Color.PURPLE);
                break;
        }
        stopMovement(); // Stop when in range
    }

    private void handleTargetOutOfRange(int tagId, double currentDistance) {
        // Set different colors for out-of-range targets
        switch(tagId) {
            case 1:
                leds.setColor(LEDSubsystem.Color.RED);
                break;
            case 2:
                leds.setColor(LEDSubsystem.Color.BLUE);
                break;
            case 3:
                leds.setColor(LEDSubsystem.Color.YELLOW);
                break;
            case 4:
                leds.setColor(LEDSubsystem.Color.ORANGE);
                break;
            default:
                leds.setColor(LEDSubsystem.Color.PURPLE);
                break;
        }

        // Move towards or away from target based on current distance
  // Calculate how far we need to move
  double distanceError = TARGET_DISTANCE - currentDistance;
  double moveSpeed = Math.copySign(TEST_SPEED, distanceError);
        
 // If distanceError is positive, we're too far away -> move forward
        // If distanceError is negative, we're too close -> move backward
        drivetrain.applyRequest(() -> drive
            .withVelocityX(moveSpeed) // Forward/backward movement
            .withVelocityY(0)         // No sideways movement
            .withRotationalRate(0));   // No rotation
    }

    private void stopMovement() {
        drivetrain.applyRequest(() -> drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        leds.setOff();
        SmartDashboard.putBoolean("Level 2 Test Running", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until cancelled
    }
}
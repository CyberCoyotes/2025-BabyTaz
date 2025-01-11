package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Command to align robot rotation to face a vision target using PID control.
 * Uses limelight horizontal offset for feedback control.
 */
public class AlignToTargetCommand extends Command {
    // Subsystem dependencies
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController alignmentPID;
    private final SwerveRequest.FieldCentric drive;
     
    // TODO: Tune these PID constants for your specific robot
    // Consider starting with just P gain and add D if needed
    private static final double kP = 0.1;  // Proportional gain
    private static final double kI = 0.0;  // Integral gain (likely not needed)
    private static final double kD = 0.01; // Derivative gain for dampening
    // TODO: Adjust max rotation speed based on testing
    private static final double maxRotationSpeed = 1.0; // Maximum rotation rate (rad/s)

    // Debugging and tuning support through Shuffleboard
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    // Live-tunable PID gains
    private final GenericEntry pGain = tab.add("P Gain", kP).getEntry();
    private final GenericEntry dGain = tab.add("D Gain", kD).getEntry();

    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.alignmentPID = new PIDController(kP, kI, kD);
        this.drive = new SwerveRequest.FieldCentric();
        
        // TODO: Adjust tolerance based on accuracy needs
        alignmentPID.setTolerance(2.0); // Acceptable error in degrees
        
        addRequirements(drivetrain);
    }

    @Override 
    public void initialize() {
        // Clear any accumulated integral term
        alignmentPID.reset();
    }

    @Override
    public void execute() {
        // Debug output
        System.out.println("Vision has target: " + vision.hasTarget());
        System.out.println("Horizontal offset: " + vision.getHorizontalOffset());
        
        // Telemetry for driver feedback and debugging
        SmartDashboard.putNumber("Target Offset", vision.getHorizontalOffset());
        
        if (vision.hasTarget()) {
            // Update PID gains from dashboard for live tuning
            alignmentPID.setP(pGain.getDouble(kP));
            alignmentPID.setD(dGain.getDouble(kD));
            
            // TODO: Consider adding target ID validation if using AprilTags
            
            // Calculate rotation correction
            // Negative offset means target is to the left, positive means to the right
            double rotationSpeed = VisionConstants.LIMELIGHT_DIRECTION * alignmentPID.calculate(vision.getHorizontalOffset(), 0);
            
            // Prevent excessive rotation speeds
            rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotationSpeed, maxRotationSpeed);
            
            // Log for debugging
            SmartDashboard.putNumber("Vision/Rotation Speed", rotationSpeed);
            
            // Apply rotation while keeping robot stationary
            drivetrain.setControl(drive
                .withVelocityX(0)    // No forward/back movement
                .withVelocityY(0)    // No left/right movement
                .withRotationalRate(rotationSpeed)); // Only rotate to target
        } else {
            // No target visible - stop movement
            drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Consider adding timeout to prevent infinite alignment attempts
        return !vision.hasTarget() || alignmentPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure robot stops moving when command ends
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
}
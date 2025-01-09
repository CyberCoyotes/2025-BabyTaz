package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import com.ctre.phoenix6.swerve.SwerveRequest;

/*
 * Simpler command that only aligns the robot's rotation to face a target using vision data
 */

 public class AlignToTargetCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController alignmentPID;
    private final SwerveRequest.FieldCentric drive;

    // TODO PID constants for rotational alignment
    private static final double kP = 0.1;
    private static final double kI = 0.0; 
    private static final double kD = 0.01;
    private static final double maxRotationSpeed = 1.0;



    // Add debugging/tuning support
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    private final GenericEntry pGain = tab.add("P Gain", kP).getEntry();
    private final GenericEntry dGain = tab.add("D Gain", kD).getEntry();
    private final double lastRotationSpeed = 0.0;


    public AlignToTargetCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.alignmentPID = new PIDController(kP, kI, kD);
        this.drive = new SwerveRequest.FieldCentric();
        
        alignmentPID.setTolerance(2.0); // 2 degrees tolerance
        
        addRequirements(drivetrain);
    }

    @Override 
    public void initialize() {
        // Reset PID controller when command starts
        alignmentPID.reset();
    }

    @Override
    public void execute() {
        System.out.println("Vision has target: " + vision.hasTarget());
        System.out.println("Horizontal offset: " + vision.getHorizontalOffset());
       // Add more detailed telemetry
        SmartDashboard.putNumber("Target Offset", vision.getHorizontalOffset());
        
        if (vision.hasTarget()) {
            // Update PID gains from dashboard (optional)
            alignmentPID.setP(pGain.getDouble(kP));
            alignmentPID.setD(dGain.getDouble(kD));
            
             // Calculate rotation speed based on horizontal offset
             double rotationSpeed = alignmentPID.calculate(vision.getHorizontalOffset(), 0);
            
             // Limit rotation speed
             rotationSpeed = MathUtil.clamp(rotationSpeed, -maxRotationSpeed, maxRotationSpeed);
             
             // Log calculated values
             SmartDashboard.putNumber("Vision/Rotation Speed", rotationSpeed);
             
 
            
            drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed));
        } else {
            // Explicitly stop if no target
            drivetrain.setControl(drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
        }
    }

    @Override
    public boolean isFinished() {
        // Command should finish when either:
        // - No target is visible
        // - Or we're within tolerance of the target
        return !vision.hasTarget() || alignmentPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all movement when command ends
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

}
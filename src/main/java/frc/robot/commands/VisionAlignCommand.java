package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionState;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionAlignCommand extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final double targetOffset;
    private final double targetDistance = 0.1; // 10cm in meters

    // Add speed scaling constants
    private static final double STRAFE_SPEED_SCALE = 0.5; // 50% speed
    private static final double FORWARD_SPEED_SCALE = 0.3; // 30% speed  
    private static final double ROTATION_SPEED_SCALE = 0.4; // 40% speed
    
    // Create PID controllers for each movement axis
    private final ProfiledPIDController strafeController;
    private final ProfiledPIDController distanceController;
    private final ProfiledPIDController rotationController;
    
    // CTRE Swerve request
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public VisionAlignCommand(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, double targetOffset) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.targetOffset = targetOffset;
        
        // Configure PID controllers with constraints
        TrapezoidProfile.Constraints translationConstraints = new TrapezoidProfile.Constraints(
            VisionConstants.MAX_TRANSLATIONAL_VELOCITY,
            VisionConstants.MAX_TRANSLATIONAL_ACCELERATION);
            
        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
            VisionConstants.MAX_ANGULAR_VELOCITY,
            VisionConstants.MAX_ANGULAR_ACCELERATION);
            
        strafeController = new ProfiledPIDController(
            VisionConstants.TRANSLATIONAL_kP,
            VisionConstants.TRANSLATIONAL_kI,
            VisionConstants.TRANSLATIONAL_kD,
            translationConstraints);
            
        distanceController = new ProfiledPIDController(
            VisionConstants.TRANSLATIONAL_kP,
            VisionConstants.TRANSLATIONAL_kI,
            VisionConstants.TRANSLATIONAL_kD,
            translationConstraints);
            
        rotationController = new ProfiledPIDController(
            VisionConstants.ANGULAR_kP,
            VisionConstants.ANGULAR_kI,
            VisionConstants.ANGULAR_kD,
            rotationConstraints);
            
        configurePIDControllers();
        addRequirements(vision, drivetrain);
    }

    private void configurePIDControllers() {
        // Set tolerances and continuous input for rotation
        strafeController.setTolerance(VisionConstants.POSITION_TOLERANCE);
        distanceController.setTolerance(VisionConstants.POSITION_TOLERANCE);
        rotationController.setTolerance(VisionConstants.ANGULAR_TOLERANCE);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        strafeController.reset(0);
        distanceController.reset(0);
        rotationController.reset(0);
    }

    @Override
    public void execute() {
        if (!vision.hasTarget()) {
            stopMovement();
            return;
        }

        // Calculate all movement components
        double horizontalOffset = vision.getHorizontalOffset() - targetOffset;
        double currentDistance = vision.getTargetDistance();
        double rotationOffset = vision.getTargetRotation();

        // Calculate velocities and apply scaling
        double strafeVelocity = strafeController.calculate(horizontalOffset, 0) * STRAFE_SPEED_SCALE;
        double forwardVelocity = distanceController.calculate(currentDistance, targetDistance) * FORWARD_SPEED_SCALE;
        double rotationRate = rotationController.calculate(rotationOffset, 0) * ROTATION_SPEED_SCALE;

        // Apply combined movement
        drivetrain.setControl(drive
            .withVelocityX(forwardVelocity)
            .withVelocityY(strafeVelocity)
            .withRotationalRate(rotationRate));     

        // Log telemetry
        updateTelemetry(horizontalOffset, currentDistance, rotationOffset);
    }

    private void updateTelemetry(double offset, double distance, double rotation) {
        SmartDashboard.putNumber("V13/HorizontalOffset", offset);
        SmartDashboard.putNumber("V13/Distance", distance);
        SmartDashboard.putNumber("V13/Rotation", rotation);
        SmartDashboard.putBoolean("V13/AtTarget", isFinished());
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() ||
               (strafeController.atGoal() && 
                distanceController.atGoal() && 
                rotationController.atGoal());
    }

    private void stopMovement() {
        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        stopMovement();
        strafeController.reset(new TrapezoidProfile.State(0, 0));
        distanceController.reset(new TrapezoidProfile.State(0, 0));
        rotationController.reset(new TrapezoidProfile.State(0, 0));
    }
}
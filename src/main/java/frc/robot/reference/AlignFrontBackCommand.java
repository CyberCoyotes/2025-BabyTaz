package frc.robot.reference;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignFrontBackCommand extends Command {
    public static enum StrafeDirection {
        LEFT(1.0),
        RIGHT(-1.0),
        FORWARD(1.0),
        BACKWARD(-1.0);

        private final double value;
        private StrafeDirection(double value) { this.value = value; }
        public double getValue() { return value; }
    }

    // Constants
    private static final double DRIVE_VELOCITY_MPS = 1.0;  // Meters per second
    private static final double DRIVE_DISTANCE_INCHES = 12.0;  // Target distance in inches
    private static final double DRIVE_DISTANCE_METERS = DRIVE_DISTANCE_INCHES * 0.0254;  // Converted to meters
    private static final double MODULE_ALIGN_DELAY = 0.1;  // Seconds to wait for module alignment

    private final CommandSwerveDrivetrain m_drive;
    private Translation2d m_startPosition;
    private final StrafeDirection m_direction;
    private final SwerveRequest.RobotCentric m_request = new SwerveRequest.RobotCentric();

    public AlignFrontBackCommand(CommandSwerveDrivetrain driveSubsystem, StrafeDirection direction) {
        m_drive = driveSubsystem;
        m_direction = direction;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_startPosition = m_drive.getState().Pose.getTranslation();
        
        // Configure strafe velocity based on direction
        double velocityMps = DRIVE_VELOCITY_MPS * m_direction.getValue();
        
        m_request.withVelocityX(velocityMps) // "Y" Change to drive forward/backward "X"
                .withVelocityY(0) // "X" Change to drive forward/backward "Y"
                .withRotationalRate(0);
                
        // Pre-align wheels
        m_drive.setControl(m_request.withVelocityY(0).withVelocityX(0));
        Timer.delay(MODULE_ALIGN_DELAY);
    }

    @Override
    public void execute() {
        m_drive.setControl(m_request);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setControl(new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        Translation2d currentPosition = m_drive.getState().Pose.getTranslation();
        double distanceTraveled = Math.abs(currentPosition.getX() - m_startPosition.getX()); // "Y" Change to drive forward/backward "X"
        return distanceTraveled >= DRIVE_DISTANCE_METERS;
    }
}
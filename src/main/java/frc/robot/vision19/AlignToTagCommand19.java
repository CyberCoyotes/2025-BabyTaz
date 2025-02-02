package frc.robot.vision19;

    import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.vision19.VisionConstants;
    import frc.robot.subsystems.CommandSwerveDrivetrain;
    import frc.robot.vision19.VisionSubsystem;
    
    public class AlignToTagCommand19 extends Command {
        private final VisionSubsystem vision;
        private final CommandSwerveDrivetrain drivetrain;
        private final PIDController xController;
        private final PIDController yController;
        private final PIDController rotationController;
        private final SwerveRequest.FieldCentric drive;
        private final int targetTagId;
    
        public AlignToTagCommand19(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain, int targetTagId) {
            this.vision = vision;
            this.drivetrain = drivetrain;
            this.targetTagId = targetTagId;
            
            xController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
            yController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
            rotationController = new PIDController(VisionConstants.ALIGN_P, VisionConstants.ALIGN_I, VisionConstants.ALIGN_D);
            
            // Configure tolerance for "at target" checking
            xController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_X);
            yController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_Y);
            rotationController.setTolerance(VisionConstants.ALIGNED_THRESHOLD_ROTATION);
            
            // Make rotation continuous
            rotationController.enableContinuousInput(-180, 180);
            
            drive = new SwerveRequest.FieldCentric()
                .withDeadband(0.1)
                .withRotationalDeadband(0.1);
    
            addRequirements(vision, drivetrain);
        }
    
        @Override
        public void initialize() {
            // Reset PID controllers
            xController.reset();
            yController.reset();
            rotationController.reset();
            
            // Enable Limelight LEDs for better tracking
            vision.setLeds(true);
        }
    
        @Override
        public void execute() {
            if (!vision.hasValidTarget() || vision.getTagId() != targetTagId) {
                // No valid target - stop moving
                drivetrain.setControl(drive.withVelocityX(0)
                                         .withVelocityY(0)
                                         .withRotationalRate(0));
                return;
            }
    
            // Calculate control outputs
            double xSpeed = xController.calculate(vision.getTargetXOffset(), 0);
            double ySpeed = yController.calculate(vision.getTargetYOffset(), 0);

            // FIXME there is no `getTargetRotation()` in vision 
            double rotationSpeed = 1; // rotationController.calculate(vision.getTargetRotation(), 0);

            
            // Apply speed limits
            xSpeed = Math.min(Math.abs(xSpeed), VisionConstants.MAX_DRIVE_VELOCITY) * Math.signum(xSpeed);
            ySpeed = Math.min(Math.abs(ySpeed), VisionConstants.MAX_DRIVE_VELOCITY) * Math.signum(ySpeed);
            rotationSpeed = Math.min(Math.abs(rotationSpeed), VisionConstants.MAX_ANGULAR_VELOCITY) * Math.signum(rotationSpeed);
    
            // Send commands to drivetrain
            drivetrain.setControl(drive.withVelocityX(xSpeed)
                                     .withVelocityY(ySpeed)
                                     .withRotationalRate(rotationSpeed));
        }
    
        @Override
        public void end(boolean interrupted) {
            // Stop moving and turn off LEDs
            drivetrain.setControl(drive.withVelocityX(0)
                                     .withVelocityY(0)
                                     .withRotationalRate(0));
            vision.setLeds(false);
        }
    
        @Override
        public boolean isFinished() {
            return vision.hasValidTarget() && 
                   vision.getTagId() == targetTagId &&
                   xController.atSetpoint() && 
                   yController.atSetpoint() && 
                   rotationController.atSetpoint();
        }
    }

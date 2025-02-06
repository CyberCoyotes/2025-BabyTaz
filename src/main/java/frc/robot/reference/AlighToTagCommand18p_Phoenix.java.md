package frc.robot.vision18;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.LimelightHelpers;

public class AlighToAprilTagCommand18_PhoenixFlavor extends Command {
        private final CommandSwerveDrivetrain drivetrain;
        private final VisionSubsystem18 vision;
        
        // Motion Magic request
        private final SwerveRequest.RobotCentric motionRequest = 
            new MotionMagicControl();
    
        public AlighToAprilTagCommand18_PhoenixFlavor(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
            this.drivetrain = drivetrain;
            this.vision = vision;
            
            // Configure Motion Magic
            var configs = new MotionMagicConfigs();
            configs.MotionMagicCruiseVelocity = 2.0; // meters per second
            configs.MotionMagicAcceleration = 4.0; // meters per second squared
            configs.MotionMagicJerk = 40.0; // meters per second cubed - smooths motion
            
            // Apply to drivetrain
            drivetrain.getConfigurator().apply(configs);
            
            addRequirements(drivetrain);
        }
    
        @Override 
        public void execute() {
            if (LimelightHelpers.getTV(vision.getName())) {
                // Get target pose
                double tx = LimelightHelpers.getTX(vision.getName());
                double ty = LimelightHelpers.getTY(vision.getName());
                double currentDistance = calculateDistance(ty);
    
                // Calculate target position relative to current pose
                Pose2d currentPose = drivetrain.getState().Pose;
                Pose2d targetPose = calculateTargetPose(currentPose, tx, currentDistance);
    
                // Apply Motion Magic control
                drivetrain.setControl(motionRequest
                    .withPosition(targetPose)
                    .withRotation(Rotation2d.fromDegrees(0)));  // Face forward
    
                // Log for tuning
                SmartDashboard.putNumber("Phoenix/TargetX", targetPose.getX());
                SmartDashboard.putNumber("Phoenix/TargetY", targetPose.getY());
                SmartDashboard.putNumber("Phoenix/CurrentDistance", currentDistance);
            } else {
                drivetrain.stopDrive();
            }
        }
    
        private Pose2d calculateTargetPose(Pose2d currentPose, double tx, double currentDistance) {
            // Convert Limelight angles to target position
            double targetDistance = 1.0; // Desired distance from tag
            double angleRadians = Math.toRadians(tx);
            
            // Calculate offset from current pose
            double xOffset = (currentDistance - targetDistance) * Math.cos(angleRadians);
            double yOffset = currentDistance * Math.sin(angleRadians);
    
            // Return target pose
            return new Pose2d(
                currentPose.getX() + xOffset,
                currentPose.getY() + yOffset,
                currentPose.getRotation()
            );
        }
    
        @Override
        public boolean isFinished() {
            if (!LimelightHelpers.getTV(vision.getName())) {
                return false;
            }
    
            Pose2d error = drivetrain.getState().Pose.minus(targetPose);
            return Math.abs(error.getX()) < 0.02 && // 2cm tolerance
                   Math.abs(error.getY()) < 0.02 &&
                   Math.abs(error.getRotation().getDegrees()) < 1.0;
        }
    }
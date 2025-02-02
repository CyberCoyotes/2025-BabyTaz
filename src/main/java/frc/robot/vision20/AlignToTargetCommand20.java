package frc.robot.vision20;

    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.vision20.VisionSubsystem;
    import frc.robot.subsystems.CommandSwerveDrivetrain;
    import frc.robot.vision20.VisionConstants;
    import edu.wpi.first.math.controller.PIDController;
    
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.math.controller.PIDController;
    import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
    
    public class AlignToTargetCommand20 extends Command {
        private final VisionSubsystem vision;
        private final CommandSwerveDrivetrain drivetrain;
        private final PIDController rotationController;
        private final PIDController distanceController;
        private final SwerveRequest.FieldCentric driveRequest;
    
        // PID Constants - tune these for your robot
        private static final double ROTATION_P = 0.015;
        private static final double ROTATION_I = 0.0;
        private static final double ROTATION_D = 0.001;
        
        private static final double DISTANCE_P = 0.02;
        private static final double DISTANCE_I = 0.0;
        private static final double DISTANCE_D = 0.0;
    
        // Tolerances
        private static final double ROTATION_TOLERANCE_DEG = 1.0;
        private static final double DISTANCE_TOLERANCE_M = 0.05;
    
        
        public AlignToTargetCommand20(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
            this.vision = vision;
            this.drivetrain = drivetrain;
            
            // Initialize PID controllers
            rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
            rotationController.setTolerance(ROTATION_TOLERANCE_DEG);
            rotationController.enableContinuousInput(-180, 180);
            
            distanceController = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
            distanceController.setTolerance(DISTANCE_TOLERANCE_M);
            
            // Initialize drive request
            driveRequest = new SwerveRequest.FieldCentric();
            
            addRequirements(vision, drivetrain);
        }
    
        @Override
        public void initialize() {
            // Reset PID controllers
            rotationController.reset();
            distanceController.reset();
            
            // Set Limelight to AprilTag pipeline
            vision.setPipeline(0);  // Assuming pipeline 0 is configured for AprilTags
            
            // Force LEDs on
            vision.setLEDMode(3);
        }
    
        @Override
        public void execute() {
            if (vision.hasValidTarget()) {
                // Get vision measurements
                double tx = vision.getHorizontalOffset();
                double distance = vision.calculateDistance();
                
                // Calculate control outputs
                double rotationOutput = rotationController.calculate(tx, 0.0);
                double distanceOutput = distanceController.calculate(distance, 1.0); // Target 1m distance
                
                // Clamp outputs for safety
                rotationOutput = MathUtil.clamp(rotationOutput, -0.5, 0.5);
                distanceOutput = MathUtil.clamp(distanceOutput, -0.3, 0.3);
                
                // Apply to drivetrain using the CTRE swerve request
                drivetrain.setControl(driveRequest
                    .withVelocityX(distanceOutput)  // Forward/backward
                    .withVelocityY(0.0)            // No strafe
                    .withRotationalRate(rotationOutput) // Rotation correction
                );
            } else {
                // No target - stop the robot
                drivetrain.stopDrive();
            }
        }
    
        @Override
        public void end(boolean interrupted) {
            // Stop the drivetrain
            drivetrain.stopDrive();
            
            // Reset Limelight LED mode
            vision.setLEDMode(1);
        }
    
        @Override
        public boolean isFinished() {
            // Command finishes when we're aligned within tolerances and have a valid target
            return vision.hasValidTarget() && 
                   rotationController.atSetpoint() &&
                   distanceController.atSetpoint();
        }
    
    
    }
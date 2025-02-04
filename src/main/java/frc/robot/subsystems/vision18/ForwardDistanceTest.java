package frc.robot.subsystems.vision18;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ForwardDistanceTest extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem18 vision;
    private final PIDController distanceController;
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private static final double TARGET_DISTANCE = 1.0; // meters

    public ForwardDistanceTest(CommandSwerveDrivetrain drivetrain, VisionSubsystem18 vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        distanceController = new PIDController(0.15, 0, 0);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            drivetrain.stopDrive();
            return;
        }

        double distance = calculateDistance(LimelightHelpers.getTY(vision.getName()));
        double forwardSpeed = -distanceController.calculate(distance, TARGET_DISTANCE); // Note negative
        
        SmartDashboard.putNumber("FwdTest/Distance", distance);
        SmartDashboard.putNumber("FwdTest/Speed", forwardSpeed);

        drivetrain.setControl(drive
            .withVelocityX(forwardSpeed)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    private double calculateDistance(double ty) {
        double cameraHeight = 0.5;  // Update these measurements!
        double targetHeight = 1.45; // 2024 AprilTag height
        double cameraAngle = 0;     // Update camera angle!
        
        return (targetHeight - cameraHeight) / 
               Math.tan(Math.toRadians(cameraAngle + ty));
    }
}
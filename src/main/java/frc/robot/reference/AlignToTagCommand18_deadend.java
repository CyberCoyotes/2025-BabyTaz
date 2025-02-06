package frc.robot.reference;

/* 
 * Value Testing 
 */


import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision18.VisionSubsystem;

public class AlignToTagCommand18_deadend extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    // Adjust PID controllers with lower gains and deadbands
    private final PIDController xController; // Controls forward/backward
    private final PIDController yController; // Controls left/right
    private final PIDController rotationController; // Controls rotation

    // Add SwerveRequest for robot-centric drive
    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();

    public AlignToTagCommand18_deadend(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        xController = new PIDController(0.0, 0.0, 0.0); // Distance control
        yController = new PIDController(0.0, 0.0, 0.0); // Lateral control
        rotationController = new PIDController(0.0, 0.0, 0.0); // Rotation control

        // Increase tolerance for testing
        xController.setTolerance(0.05); // 5cm
        yController.setTolerance(0.05); // 5cm
        rotationController.setTolerance(Math.toRadians(2.0));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        // DEBUGGING ONLY
        // 1. Get raw targeting data and verify basic tracking
        boolean hasTarget = LimelightHelpers.getTV(vision.getName());
        double tx_raw = LimelightHelpers.getTX(vision.getName());
        double ty_raw = LimelightHelpers.getTY(vision.getName());
        double ta_raw = LimelightHelpers.getTA(vision.getName());

        // DEBUGGING ONLY
        // Log ALL raw values to compare with Limelight dashboard
        SmartDashboard.putBoolean("V18/HasTarget", hasTarget);
        SmartDashboard.putNumber("V18/TX_Raw", tx_raw);
        SmartDashboard.putNumber("V18/TY_Raw", ty_raw);
        SmartDashboard.putNumber("V18/TA_Raw", ta_raw);

        if (!hasTarget) {
            drivetrain.stopDrive();
            return;
        }

        // Use raw values for control
        double ySpeed = -yController.calculate(tx_raw, 0);

        // Log control values
        SmartDashboard.putNumber("V18/YSpeed_Raw", ySpeed);
        SmartDashboard.putNumber("V18/YController_SetPoint", 0);
        SmartDashboard.putNumber("V18/YController_Error", -tx_raw);

        // Apply speed limits and log
        double maxSpeed = 0.5;
        ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);
        SmartDashboard.putNumber("V18/YSpeed_Clamped", ySpeed);

        // For initial testing, ONLY do Y-axis alignment
        drivetrain.setControl(robotCentric
                .withVelocityX(0) // Zero for testing
                .withVelocityY(ySpeed)
                .withRotationalRate(0)); // Zero for testing

        /* DEBUGGIING ONLY */
        // Get raw values
        /*
         * boolean hasTarget = LimelightHelpers.getTV(vision.getName());
         * 
         * // Get value multiple ways for comparison
         * double tx_helper = LimelightHelpers.getTX(vision.getName());
         * double tx_direct = NetworkTableInstance.getDefault()
         * .getTable(vision.getName())
         * .getEntry("tx")
         * .getDouble(0.0);
         * 
         * // Log all versions
         * SmartDashboard.putBoolean("V18/HasTarget_Cmd", hasTarget);
         * SmartDashboard.putNumber("V18/TX_Helper_Cmd", tx_helper);
         * SmartDashboard.putNumber("V18/TX_Direct_Cmd", tx_direct);
         */
    }

    // Modified distance calculation for accuracy
    private double calculateDistance(double ty) {
        double limelightHeightMeters = Units.inchesToMeters(12.0); // FIXME Verify this height
        double limelightMountAngleDegrees = 0.0; // Updated for front mount
        double targetHeightMeters = Units.inchesToMeters(7); // FIXME Verify this target height
        // | 7" is 0.1778 meters |

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + ty);
        return (targetHeightMeters - limelightHeightMeters) / Math.tan(angleToGoalRadians);
    }

    //
    @Override
    public boolean isFinished() {
        if (!LimelightHelpers.getTV(vision.getName())) {
            return false;
        }

        return xController.atSetpoint() &&
                yController.atSetpoint() &&
                rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDrive();
    }
}

/*
 * Monday night adjustments
 * https://claude.ai/chat/cbf2369b-de90-4674-8839-5b0d91dab2a2
 * 
 * Based on your description and the code shown, there are several issues to
 * address:
 * 
 * First, there's a duplicate control call in AlignToAprilTagCommand18.execute()
 * - you're setting the control twice which could cause issues.
 * The robot maintaining a steady TY while TX becomes more negative suggests the
 * distance control (X axis) isn't responding properly and the robot is moving
 * in an arc pattern instead of correcting its distance.
 */

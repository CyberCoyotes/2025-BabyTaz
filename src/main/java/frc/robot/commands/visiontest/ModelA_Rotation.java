package frc.robot.commands.visiontest;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.TunableVisionConstants;
import org.littletonrobotics.junction.Logger;

/**
 * ===========================================================================
 * MODEL A: ROTATIONAL ALIGNMENT ONLY
 * ===========================================================================
 *
 * Adapted from Limelight "Aiming with Servoing" example.
 *
 * PURPOSE:
 * - Rotates the robot to center an AprilTag on the robot's Y-axis (tx = 0)
 * - Robot does NOT move forward/backward or strafe
 * - Pure rotation-only alignment
 *
 * HOW IT WORKS:
 * 1. Reads TX (horizontal offset) from Limelight
 * 2. Uses proportional control to calculate rotation speed
 * 3. Adds minimum speed to overcome static friction (servoing)
 * 4. Applies rotation to drivetrain
 *
 * SUCCESS CRITERIA:
 * - TX is within tolerance (default: 1.5 degrees)
 *
 * TELEMETRY OUTPUT:
 * - VisionTest/ModelA/Status: Current state (SEARCHING, ALIGNING, ALIGNED)
 * - VisionTest/ModelA/TX: Horizontal offset in degrees
 * - VisionTest/ModelA/RotationSpeed: Applied rotation rate
 * - VisionTest/ModelA/AtTarget: Boolean - true when aligned
 *
 * BUTTON BINDING: Shuffleboard "Model A: Rotation Only" button
 */
public class ModelA_Rotation extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

    // PID Controller for rotation
    private final PIDController rotationPID;

    // NetworkTables for fast telemetry output
    private final NetworkTable telemetryTable;

    public ModelA_Rotation(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        // Initialize PID controller with constants from VisionConstants
        rotationPID = new PIDController(
            VisionConstants.ModelA.ROTATION_KP,
            VisionConstants.ModelA.ROTATION_KI,
            VisionConstants.ModelA.ROTATION_KD
        );
        rotationPID.setTolerance(VisionConstants.ModelA.ROTATION_TOLERANCE_DEGREES);
        rotationPID.setSetpoint(VisionConstants.TARGET_TX_CENTERED);
        rotationPID.enableContinuousInput(-180, 180);

        // Setup NetworkTables for telemetry
        telemetryTable = NetworkTableInstance.getDefault()
            .getTable(VisionConstants.NTKeys.MODEL_A_PREFIX.replace("/", ""));

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        // Set vision mode in subsystem
        vision.setVisionMode(VisionSubsystem.VisionMode.ROTATION_ONLY);
        vision.setAlignmentState(VisionSubsystem.AlignmentState.SEARCHING);
        logStatus("STARTED");
    }

    @Override
    public void execute() {
        // Update PID gains from dashboard if changed
        updatePIDGains();

        // Check for valid target
        if (!vision.hasTarget()) {
            // Update state machine
            if (vision.getAlignmentState() == VisionSubsystem.AlignmentState.ALIGNING ||
                vision.getAlignmentState() == VisionSubsystem.AlignmentState.ALIGNED) {
                vision.setAlignmentState(VisionSubsystem.AlignmentState.LOST_TARGET);
            } else {
                vision.setAlignmentState(VisionSubsystem.AlignmentState.SEARCHING);
            }
            stopRobot();
            logTelemetry(0.0, 0.0, false);
            return;
        }

        // Get horizontal offset from Limelight
        double tx = vision.getTX();

        // Calculate rotation speed using proportional control
        // Limelight "Aiming with Servoing" approach:
        // rotationSpeed = kP * tx
        double rotationSpeed = VisionConstants.ROTATION_DIRECTION_MULTIPLIER * rotationPID.calculate(tx);

        // Add minimum speed to overcome static friction (servoing)
        // This prevents the robot from getting "stuck" near the target
        double minSpeed = TunableVisionConstants.ModelA.getMinRotationSpeed();
        if (Math.abs(rotationSpeed) > 0.01 && Math.abs(rotationSpeed) < minSpeed) {
            rotationSpeed = Math.copySign(minSpeed, rotationSpeed);
        }

        // Apply speed limits
        double maxSpeed = TunableVisionConstants.ModelA.getMaxRotationSpeed();
        rotationSpeed = MathUtil.clamp(rotationSpeed, -maxSpeed, maxSpeed);

        // Check if aligned
        boolean atTarget = rotationPID.atSetpoint();

        // Update state machine based on alignment status
        vision.setAlignmentState(atTarget ?
            VisionSubsystem.AlignmentState.ALIGNED :
            VisionSubsystem.AlignmentState.ALIGNING);

        // Apply to drivetrain - rotation only, no translation
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(rotationSpeed));

        // Log telemetry
        logTelemetry(tx, rotationSpeed, atTarget);
    }

    /**
     * Updates PID gains from tunable constants if they've changed.
     * Called every execute() cycle to allow live tuning.
     */
    private void updatePIDGains() {
        if (TunableVisionConstants.ModelA.anyPIDHasChanged()) {
            rotationPID.setPID(
                TunableVisionConstants.ModelA.getRotationKp(),
                TunableVisionConstants.ModelA.getRotationKi(),
                TunableVisionConstants.ModelA.getRotationKd()
            );
        }

        if (TunableVisionConstants.ModelA.rotationToleranceHasChanged()) {
            rotationPID.setTolerance(TunableVisionConstants.ModelA.getRotationTolerance());
        }
    }

    private void stopRobot() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withRotationalRate(0.0));
    }

    private void logTelemetry(double tx, double rotationSpeed, boolean atTarget) {
        // Get current state from subsystem
        String status = vision.getAlignmentState().name();

        // NetworkTables output (fast, reliable)
        telemetryTable.getEntry("Status").setString(status);
        telemetryTable.getEntry("TX").setDouble(tx);
        telemetryTable.getEntry("RotationSpeed").setDouble(rotationSpeed);
        telemetryTable.getEntry("AtTarget").setBoolean(atTarget);
        telemetryTable.getEntry("TagID").setDouble(vision.getTagID());

        // AdvantageKit logging (for replay)
        Logger.recordOutput("VisionTest/ModelA/Status", status);
        Logger.recordOutput("VisionTest/ModelA/TX", tx);
        Logger.recordOutput("VisionTest/ModelA/RotationSpeed", rotationSpeed);
        Logger.recordOutput("VisionTest/ModelA/AtTarget", atTarget);
        Logger.recordOutput("VisionTest/ModelA/TagID", vision.getTagID());
    }

    private void logStatus(String status) {
        telemetryTable.getEntry("CommandStatus").setString(status);
        Logger.recordOutput("VisionTest/ModelA/CommandStatus", status);
    }

    @Override
    public boolean isFinished() {
        // Command runs until manually stopped (whileTrue binding)
        // Or return true when aligned if you want auto-finish:
        // return vision.isAligned();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        stopRobot();
        // Reset state machine when command ends
        vision.resetStateMachine();
        logStatus(interrupted ? "INTERRUPTED" : "COMPLETED");
    }
}

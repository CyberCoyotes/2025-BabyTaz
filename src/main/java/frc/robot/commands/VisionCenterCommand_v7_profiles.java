package frc.robot.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.VisionSubsystem;

public class VisionCenterCommand_v7_profiles extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;
    private final DynamicMotionMagicVoltage rotationRequest;
    private final DynamicMotionMagicVoltage translationRequest;
    private final SwerveRequest.RobotCentric drive;

    // Motion parameters for rotation and translation
    private static final double ROT_VELOCITY = 80.0;  
    private static final double ROT_ACCEL = 160.0;    
    private static final double ROT_JERK = 1600.0;    

    private static final double TRANS_VELOCITY = 2.0; // m/s  
    private static final double TRANS_ACCEL = 4.0;    // m/s^2
    private static final double TRANS_JERK = 8.0;     // m/s^3

    private final BaseStatusSignal[] signals;

    public VisionCenterCommand_v7_profiles(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        
        rotationRequest = new DynamicMotionMagicVoltage(0, ROT_VELOCITY, ROT_ACCEL, ROT_JERK)
            .withSlot(0)
            .withEnableFOC(true);
            
        translationRequest = new DynamicMotionMagicVoltage(0, TRANS_VELOCITY, TRANS_ACCEL, TRANS_JERK)
            .withSlot(1)
            .withEnableFOC(true);
            
        drive = new SwerveRequest.RobotCentric(); // Use default type

        // Get signals from Pigeon2
        var pigeon = drivetrain.getPigeon2();
        signals = new BaseStatusSignal[] {
            pigeon.getYaw(),
            pigeon.getAngularVelocityZDevice()
        };
        BaseStatusSignal.setUpdateFrequencyForAll(100, signals);
    
    }
    
    private void configureGains() {
        // TODO Rotation gains (Slot 0)
        var rotationGains = new Slot0Configs()
            .withKS(0.25) // 0.1?
            .withKV(0.12)
            .withKP(4.8) // 2?
            .withKI(0.0)
            .withKD(0.1); // 0.05?
            
        // TODO Translation gains (Slot 1)
        var translationGains = new Slot1Configs()
            .withKS(0.2)
            .withKV(0.15)
            .withKP(3.0)
            .withKI(0.0)
            .withKD(0.05);

            // Consider applying the gains to the drivetrain based on drive motors OR turning motors
            // Apply the gains to the drivetrain

            drivetrain.getConfigurator().apply(rotationGains);

    }


    @Override
    public void execute() {
        BaseStatusSignal.waitForAll(0.1, signals);

        if (!vision.hasTarget()) {
            drivetrain.stop();
            return;
        }

        double targetAngle = calculateTargetAngle();
        double targetDistance = calculateTargetDistance();

        drivetrain.setControl(drive
            .withVelocityX(translationRequest.withPosition(targetDistance))
            .withVelocityY(0)
            .withRotationalRate(rotationRequest.withPosition(targetAngle)));

        logTelemetry(targetAngle, targetDistance);
    }

    private double calculateTargetAngle() {
        double currentAngle = drivetrain.getYaw().getValue();
        double latencyCompensation = drivetrain.getYawRate() * (vision.getPipelineLatency() / 1000.0);
        return currentAngle + vision.getHorizontalOffset() + latencyCompensation;
    }

    private double calculateTargetDistance() {
        // Use Limelight ty or ta value to determine distance
        double currentDistance = vision.getDistanceToTarget();
        double desiredDistance = 1.0; // meters from target
        return desiredDistance - currentDistance;
    }

    @Override
    public boolean isFinished() {
        return !vision.hasTarget() || 
               (Math.abs(vision.getHorizontalOffset()) < 1.0 && 
                Math.abs(vision.getDistanceToTarget() - 1.0) < 0.1);
    }

    private void logTelemetry(double targetAngle, double targetDistance) {
        SmartDashboard.putNumber("V7/TargetAngle", targetAngle);
        SmartDashboard.putNumber("V7/TargetDistance", targetDistance);
    }
}

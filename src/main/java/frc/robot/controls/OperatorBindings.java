package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Operator controller bindings for BabyTaz prototype.
 *
 * Shooter controls:
 *   Right Trigger (hold) — Spin flywheels at dashboard RPM
 *   Left Trigger (hold)  — Eject / reverse flywheels
 *   A button (press)     — Stop shooter
 */
public class OperatorBindings {
    private final CommandXboxController operator;
    private final ShooterSubsystem shooter;

    public OperatorBindings(CommandXboxController operator, ShooterSubsystem shooter) {
        this.operator = operator;
        this.shooter = shooter;
        configureBindings();
    }

    private void configureBindings() {
        // Hold right trigger to spin at dashboard-tunable RPM
        operator.rightTrigger().whileTrue(ShooterCommands.runFromDashboard(shooter));

        // Hold left trigger to reverse (eject) while held
        operator.leftTrigger().whileTrue(ShooterCommands.runAtRPM(shooter, -2000));

        // A button = emergency stop
        operator.a().onTrue(ShooterCommands.stop(shooter));
    }
}

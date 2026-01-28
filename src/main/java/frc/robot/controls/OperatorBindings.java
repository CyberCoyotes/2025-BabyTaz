package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Operator controller bindings for BabyTaz prototype.
 *
 * Shooter controls:
 *   Right Trigger (hold) — Spin flywheels and indexer at dashboard values
 *   Left Trigger (hold)  — Eject / reverse flywheels and indexer
 *   A button (press)     — Stop shooter and indexer
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
        // Hold right trigger to spin flywheels and indexer at dashboard-tunable values
        operator.rightBumper().whileTrue(ShooterCommands.runWithIndexer(shooter));

        // Hold left trigger to reverse (eject) flywheels and indexer while held
        operator.leftBumper().whileTrue(ShooterCommands.runAtRPMWithIndexer(shooter, -2000));

        // A button = emergency stop (flywheels and indexer)
        operator.a().onTrue(ShooterCommands.stop(shooter));
    }
}

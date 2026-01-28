package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Factory class for shooter commands on BabyTaz prototype.
 * All commands are static factories — no instantiation needed.
 */
public class ShooterCommands {

    private ShooterCommands() {} // Prevent instantiation

    /**
     * Spin flywheels at whatever RPM is currently set on the Elastic dashboard.
     * Runs until cancelled (whileTrue binding).
     */
    public static Command runFromDashboard(ShooterSubsystem shooter) {
        return Commands.startEnd(
                shooter::run,
                shooter::stop,
                shooter)
                .withName("Shooter: Run (Dashboard RPM)");
    }

    /**
     * Spin flywheels at a specific RPM. Also updates the dashboard value.
     * Runs until cancelled.
     */
    public static Command runAtRPM(ShooterSubsystem shooter, double rpm) {
        return Commands.startEnd(
                () -> shooter.runAtRPM(rpm),
                shooter::stop,
                shooter)
                .withName("Shooter: Run @ " + (int) rpm + " RPM");
    }

    /**
     * Stop the shooter. Instant command.
     */
    public static Command stop(ShooterSubsystem shooter) {
        return Commands.runOnce(shooter::stop, shooter)
                .withName("Shooter: Stop");
    }

    /**
     * Reverse the flywheels briefly for clearing jams.
     * Runs at -2000 RPM for the specified duration, then stops.
     */
    public static Command eject(ShooterSubsystem shooter, double seconds) {
        return Commands.sequence(
                Commands.runOnce(() -> shooter.runAtRPM(-2000), shooter),
                Commands.waitSeconds(seconds),
                Commands.runOnce(shooter::stop, shooter))
                .withName("Shooter: Eject");
    }
}

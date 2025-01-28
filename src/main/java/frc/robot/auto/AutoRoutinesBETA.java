package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/* Use this java file to hold AutoRoutines if other programmers are also writing AutoRoutines 
 * It will also store quick reference documentation for the AutoRoutines
*/
public class AutoRoutinesBETA {
    // Dependencies
    private final AutoFactory m_factory;
    private CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem m_turret;
    LEDSubsystem m_leds;
    VisionSubsystem m_vision;

    // Constants
    // Seconds for drivetrain to pause while scoring in autonomous
    private final double scoreDelayLow = 0.5; // seconds
    private final double scoreDelay = 1.0; // seconds
    private final double doneDriving = 14; // seconds

    /*
     * Use as dumb, fixed timer to pause while loading a CHORAL
     * This should be replaced with a sensor-based trigger in the future
     * This doubles a max time to wait for a game piece in the event of a sensor
     * failure
     */
    private final double maxLoadDelay = 2.0; // seconds
    private final double minLoadDelay = 0.2; // seconds

    // Constructor to receive dependencies
    public AutoRoutinesBETA(AutoFactory factory, CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_turret = new TurretSubsystem(27);
        // m_elevator = new ElevatorSubsystem(); // Competition robot only
        // m_intake = new IntakeSubsystem(); // Competition robot only
        // m_score = new ScoreSubsystem(); // Competition robot only
        m_vision = new VisionSubsystem("limelight", m_leds);
        m_leds = new LEDSubsystem();

    }

    
    /*
     * CTRE Example Reference
     * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/
     * SwerveWithChoreo/src/main/java/frc/robot/AutoRoutines.java
     * 
     * Choreo Example Reference
     * Choreo Project is "Tests.chor"
     * Trajectory is "SimplePath.traj"
     * 
     */

    // TODO Test this feature
    /*
     * Auto bindings are used to bind event markers in trajectories made by the
     * AutoFactory to commands.
     * Commands added to the AutoFactory using bind exhibit the same behavior as
     * those bound to AutoTrajectory.atTime(String),
     * except they are applied globally across all routines.
     * This is useful if you have simpler actions that you want to trigger in any
     * trajectory without much thought.
     * Warning
     * Even if a marker is bound individually in an AutoTrajectory.atTime(String)
     * trigger, the global binding will still run, and cannot be disabled for a
     * single marker.
     * ```
     * m_autoFactory
     * .bind("intake", intakeSubsystem.intake())
     * .bind("score", scoringSubsystem.score());
     * ```
     * my example
     * 
     * auto
     * 
     */

    // TODO Add global commands for event triggers in Choreo
    // m_factory
    // Will run TurnTurretClockwise whenever "turret" marker is hit
    // .bind("turret", turret.turnClockwise());
    // another approach
    // .bind("turret", Commands.run(() -> turret.turnClockwise(0.5)))
    // These run!
    // .bind("scoreL1", turret.turnClockwise()) // Comment out for in method testing
    // with atTime()
    // .bind("scoreL2", turret.turnCounterClockwise());
    // ;

    /*
     * CHOREO EXAMPLE
     * // Load the routine's trajectories
     * AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
     * AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");
     * 
     * // When the routine begins, reset odometry and start the first trajectory
     * routine.active().onTrue(
     * Commands.sequence(
     * pickupTraj.resetOdometry(),
     * pickupTraj.cmd()
     * )
     * );
     * 
     * // Starting at the event marker named "intake", run the intake
     * pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());
     * 
     * // When the trajectory is done, start the next trajectory
     * pickupTraj.done().onTrue(scoreTraj.cmd());
     * 
     * // While the trajectory is active, prepare the scoring subsystem
     * scoreTraj.active().whileTrue(scoringSubsystem.getReady());
     * 
     * // When the trajectory is done, score
     * scoreTraj.done().onTrue(scoringSubsystem.score());
     * 
     * return routine;
     */
}

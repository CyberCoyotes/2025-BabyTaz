package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.TurretSubsystem;

/*
 * CTRE Example Reference
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithChoreo/src/main/java/frc/robot/AutoRoutines.java
 * 
 * Choreo Example Reference
 * Choreo Project is "Tests.chor"
 * Trajectory is "SimplePath.traj"
 * 
 */

public class AutoRoutines {
    private final AutoFactory m_factory;
    private CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final TurretSubsystem m_turret;

    // Constructor to receive dependencies
    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_turret = new TurretSubsystem(27);
        // Command TurnTurretClockwise = new TurnTurretClockwise(m_turret);
    }


    // TODO Test this feature
    /*
     * Auto bindings are used to bind event markers in trajectories made by the AutoFactory to commands.
     * Commands added to the AutoFactory using bind exhibit the same behavior as those bound to AutoTrajectory.atTime(String),
     * except they are applied globally across all routines.
     * This is useful if you have simpler actions that you want to trigger in any trajectory without much thought.
     * Warning
     * Even if a marker is bound individually in an AutoTrajectory.atTime(String) trigger, the global binding will still run, and cannot be disabled for a single marker.
    ```
    autoFactory
    .bind("intake", intakeSubsystem.intake())
    .bind("score", scoringSubsystem.score());
    ```
    my example

    auto
    
     */



    /*
    CHOREO EXAMPLE 
     * // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory 
    routine.active().onTrue(
        Commands.sequence(
            pickupTraj.resetOdometry(),
            pickupTraj.cmd()
        )
    );

    // Starting at the event marker named "intake", run the intake 
    pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
     */
    public AutoRoutine threeMetersPlus() {

        // Step 1: Create the routine container
        final AutoRoutine routine = m_factory.newRoutine("ThreeMetersPlus");

        // Step 2: Load trajectories
        // final AutoTrajectory startScoreA = routine.trajectory("ThreeMetersPlus");
        // final AutoTrajectory startScoreB = routine.trajectory("ThreeMetersPlus");
        final AutoTrajectory startScoreK = routine.trajectory("ThreeMetersPlus");
        // final AutoTrajectory topStationFromK = routine.trajectory("ThreeMetersPlus");

        routine.active().onTrue(
                Commands.sequence(
                        startScoreK.resetOdometry(), // Always reset odometry first
                        startScoreK.cmd() // Follow first path
                        // ,Commands.waitSeconds(2.0)   // Pause for 2 seconds
                ));


        // TODO comment out because trying auto bindings from robot container on things like "turret"
        // Step 4: Add trigger-based behaviors

        // Approach using a Command class
        // startScoreK.atTime("turret").onTrue(new TurnTurretClockwise(m_turret));

        // Approach using a lambda function referencing the turret subsystem and a method
        // startScoreK.atTime("turret").onTrue(m_turret.turnClockwise());

        // When turret is done, start second path
        // startScoreK.done().onTrue(kTopStation.cmd());

        return routine;
    }

    public AutoRoutine scoreK1() {
        final AutoRoutine routine = m_factory.newRoutine("StartTopScoreK");
        final AutoTrajectory startTopScoreK = routine.trajectory("StartTopScoreK");

        routine.active().onTrue(
                startTopScoreK.resetOdometry()
                        .andThen(startTopScoreK.cmd()));
        return routine;
    }

    public AutoRoutine scoreK2() {
        final AutoRoutine routine = m_factory.newRoutine("StartTopScoreK");
        final AutoTrajectory startTopScoreK = routine.trajectory("StartTopScoreK");

        routine.active().onTrue(
                startTopScoreK.resetOdometry()
                        .andThen(startTopScoreK.cmd()));
        return routine;
    }

    public AutoRoutine scoreK4() {
        final AutoRoutine routine = m_factory.newRoutine("StartTopScoreK");
        final AutoTrajectory startTopScoreK = routine.trajectory("StartTopScoreK");

        routine.active().onTrue(
                startTopScoreK.resetOdometry()
                        .andThen(startTopScoreK.cmd()));
        return routine;
    }

    
    public AutoRoutine threeMeters() {

        // Creates a new routine with the name "exampleRoutine"
        final AutoRoutine routine = m_factory.newRoutine("Three Meters");

        // Assign a variable to path/traj in the Choreo Project
        final AutoTrajectory threeMeters = routine.trajectory("ThreeMeters");

        routine.active().onTrue(
                threeMeters.resetOdometry()
                        .andThen(threeMeters.cmd()));
        return routine;
    }

    public AutoRoutine testEvents() {

        // Create the routine container
        final AutoRoutine routine = m_factory.newRoutine("TestingEvents");

        // Load trajectories
        final AutoTrajectory pathOneSeg1 = routine.trajectory("TestingEvents",0);

        // First split is actually at waypoint 2, but I needed to "pick up" the path again at the point prior for the routine to continue
        final AutoTrajectory pathOneSeg2 = routine.trajectory("TestingEvents",1); 

        // Second split is actually at waypoint 2, but I needed to "pick up" the path again at the point prior for the routine to continue
        final AutoTrajectory pathOneSeg3 = routine.trajectory("TestingEvents",2); 

        // Define entry point using routine.active()
        routine.active().onTrue(
                Commands.sequence(
                        pathOneSeg1.resetOdometry(), // Always reset odometry first
                        pathOneSeg1.cmd(),
                        m_drivetrain.stop().withTimeout(1),
                        pathOneSeg2.cmd(),
                        m_drivetrain.stop().withTimeout(1),
                        pathOneSeg3.cmd()
                ));

        /* This works, mostly as expected.
        See also autobindings in RobotContainer for repetitive commands.
        Robot doesn't stop at waypoint 2 as expected */
        
        // Add trigger-based behaviors
        pathOneSeg1.atTime("scoreL1").onTrue(m_turret.turnClockwise());
        pathOneSeg2.atTime("scoreL2").onTrue(m_turret.turnCounterClockwise());

        // When turret is done, start second path
        // testingEvents.done().onTrue(testingEvents.cmd());

        return routine;
    }

    /*
    public AutoRoutine startTopScoreK() {
        final AutoRoutine routine = m_factory.newRoutine("Start Top Score K");

        final AutoTrajectory startTopScoreK = routine.trajectory("StartTopScoreK");

        routine.active().onTrue(
                startTopScoreK.resetOdometry()
                        .andThen(startTopScoreK.cmd()));
        return routine;
    } */

}
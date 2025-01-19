package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.commands.TurnTurretClockwise;

import edu.wpi.first.wpilibj2.command.Command;

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
    private final TurretSubsystem m_turret;

    // Constructor to receive dependencies
    public AutoRoutines(AutoFactory factory, TurretSubsystem turret) {
        m_factory = factory;
        m_turret = new TurretSubsystem(27);
        Command TurnTurretClockwise = new TurnTurretClockwise(m_turret);

    }


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
        // final AutoTrajectory firstPath = routine.trajectory("PathOne");
        // final AutoTrajectory secondPath = routine.trajectory("PathTwo");

        final AutoTrajectory myPath = routine.trajectory("ThreeMetersPlus");
        // final AutoTrajectory threeMetersPlus = routine.trajectory("ThreeMetersPlus");

        routine.active().onTrue(
                Commands.sequence(
                        myPath.resetOdometry(), // Always reset odometry first
                        myPath.cmd() // Follow first path

                ));

        // Step 4: Add trigger-based behaviors
        // When first path is done, start turret command

        // Starting at the event marker named "intake", run the intake 
        // TODO Test 1
        // myPath.atTime("turret").onTrue(new RunCommand(() -> m_turret.turnClockwise(-0.5), m_turret));

        // TODO Test 2
        // myPath.atTime("turret").onTrue(m_turret.turnClockwiseNonVoid());


        // TODO Test 3
        myPath.atTime("turret").onTrue(new TurnTurretClockwise(m_turret));


        // firstPath.done().onTrue(new TurnTurretClockwise(m_turret));

        // When turret is done, start second path
    // m_turret.done().onTrue(secondPath.cmd());

        return routine;
    }

    // You can add more autonomous routines as additional methods
    // public AutoRoutine anotherAuto() {
    // Implementation for another autonomous routine
    // }

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
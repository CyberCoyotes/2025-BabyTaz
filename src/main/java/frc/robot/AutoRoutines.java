package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import frc.robot.commands.TurnTurretClockwise;
import frc.robot.subsystems.turret.TurretSubsystem;

public class AutoRoutines {
    private final AutoFactory m_factory;
    private final TurretSubsystem m_turret;

    public AutoRoutines(AutoFactory factory, TurretSubsystem turret) {
        m_factory = factory;
        m_turret = new TurretSubsystem(27);
    }


  public AutoRoutine threeMeters() {

        // This is the name of the Choreo Project
        final AutoRoutine cProject = m_factory.newRoutine("BabyTazLV");

        // This is the name of the path/traj in the Choreo Project
        final AutoTrajectory threeMeters = cProject.trajectory("ThreeMeters");


        
        cProject.active().onTrue(
            threeMeters.resetOdometry()
                .andThen(threeMeters.cmd())
        );
        return cProject;
    }


    public AutoRoutine threeMetersPlus() {

        // This is the name of the Choreo Project
        final AutoRoutine cProject = m_factory.newRoutine("BabyTazLV");
        
        // This is the name of the path/traj in the Choreo Project
        final AutoTrajectory threeMetersPlus = cProject.trajectory("ThreeMetersPlus");
                
        cProject.active().onTrue(
            threeMetersPlus.resetOdometry()
                .andThen(threeMetersPlus.cmd()
                    // This works, not exactly what I envisioned
                    
                    // .deadlineWith(new TurnTurretClockwise(m_turret))
                    // .beforeStarting(() -> {})
                    // .andThen(() -> {})
                )
        );

        return cProject;
    }
}
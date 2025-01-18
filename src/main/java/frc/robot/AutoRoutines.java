package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
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
                .andThen(threeMetersPlus.cmd())
        );
        return cProject;
    }
}
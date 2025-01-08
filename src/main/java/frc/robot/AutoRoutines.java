package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine twoMeters() {
        final AutoRoutine routine = m_factory.newRoutine("Two Meters");
        final AutoTrajectory twoMeters = routine.trajectory("TwoMeters");

        routine.active().onTrue(
            twoMeters.resetOdometry()
                .andThen(twoMeters.cmd())
        );
        return routine;
    }
}
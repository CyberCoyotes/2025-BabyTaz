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

public class AutoRoutines {
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
    Use as  dumb, fixed timer to pause while loading a CHORAL
    This should be replaced with a sensor-based trigger in the future
    This doubles a max time to wait for a game piece in the event of a sensor failure
    */ 
    private final double maxLoadDelay = 2.0; // seconds
    private final double minLoadDelay = 0.2; // seconds

    // Constructor to receive dependencies
    public AutoRoutines(AutoFactory factory, CommandSwerveDrivetrain drivetrain, TurretSubsystem turret) {
        m_factory = factory;
        m_drivetrain = drivetrain;
        m_turret = new TurretSubsystem(27);
        // m_elevator = new ElevatorSubsystem(); // Competition robot only
        // m_intake = new IntakeSubsystem(); // Competition robot only
        // m_score = new ScoreSubsystem(); // Competition robot only
        m_vision = new VisionSubsystem("limelight", m_drivetrain, m_leds);
        m_leds = new LEDSubsystem();

        }

    /* Drive forward from the center position, or any position, and stop */
    public AutoRoutine driveForward() {
        final AutoRoutine routine = m_factory.newRoutine("DriveForward");
        final AutoTrajectory driveForward = routine.trajectory("DriveForward");

        routine.active().onTrue(
                Commands.sequence(
                        driveForward.resetOdometry(), // Always reset odometry first
                        driveForward.cmd() // Follow the path
                        // driveForward.done() // TODO add a done command
                ));

        return routine;
    }

        /* Drive forward from the center position, stop, and score */
        public AutoRoutine scoreCenter() {
            final AutoRoutine routine = m_factory.newRoutine("ScoreCenter");
            final AutoTrajectory driveForward = routine.trajectory("ScoreCenter");
    
            routine.active().onTrue(
                    Commands.sequence(
                            driveForward.resetOdometry(), // Always reset odometry first
                            driveForward.cmd() // Follow the path
                            // driveForward.done() // TODO add a done command
                    ));
            // TODO Add event markers for scoring L1 as a last resort
            return routine;
        }

                /* Drive forward from the center position, stop, and score */
                public AutoRoutine scoreCenterL1() {
                    final AutoRoutine routine = m_factory.newRoutine("ScoreCenter");
                    final AutoTrajectory driveForward = routine.trajectory("ScoreCenter");
            
                    routine.active().onTrue(
                            Commands.sequence(
                                    driveForward.resetOdometry(), // Always reset odometry first
                                    driveForward.cmd() // Follow the path
                                    // driveForward.done() // TODO add a done command
                            ));
                    // TODO Add event markers for scoring L1 as a last resort
                    return routine;
                }

    /* Start at the top, score K, and load another CHORAL */
    public AutoRoutine topK() {

        final AutoRoutine routine = m_factory.newRoutine("TopK");
        final AutoTrajectory topK = routine.trajectory("TopK", 0);
        final AutoTrajectory loadK = routine.trajectory("TopK", 1);
        
        routine.active().onTrue(
            Commands.sequence(
                topK.resetOdometry(),
                // Start to Score K
                topK.cmd(),
                // Stop and Score
                m_drivetrain.stop().withTimeout(scoreDelay),
                // Drive to Load station from K
                loadK.cmd()
            ));

        
        return routine;
    }
    public AutoRoutine topKversion2() {

        final AutoRoutine routine = m_factory.newRoutine("TopK");
        final AutoTrajectory topK = routine.trajectory("TopK", 0);
        final AutoTrajectory loadK = routine.trajectory("TopK", 1);
        
        routine.active().onTrue(
                 topK.resetOdometry()
                .andThen(topK.cmd())
                .andThen(m_drivetrain.stop().withTimeout(scoreDelay))
            );

        
        return routine;
    }

    public AutoRoutine scoreDoubleK() {
        final AutoRoutine routine = m_factory.newRoutine("ScoreDoubleK");
        final AutoTrajectory topK = routine.trajectory("TopK",0);
        final AutoTrajectory loadK = routine.trajectory("TopK",1);
        final AutoTrajectory scoreK = routine.trajectory("scoreK",0);
        //final AutoTrajectory loadK = routine.trajectory("TopK",1);

        // routine.active().onTrue(
 
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

    public AutoRoutine testEvents() {

        // Create the routine container
        final AutoRoutine routine = m_factory.newRoutine("testEvents");

        // Load first trajectory/path. If using splits, then add a comma and 0 for the first segment of the path
        final AutoTrajectory pathOneScore = routine.trajectory("PathOneTest",0);
        
        /* 
        Load the next trajectory/path -OR-
        If using splits as is the case here, you can load segment 1 for the next segment of the first path
        */ 
        final AutoTrajectory pathOneLoad = routine.trajectory("PathOneTest",1); 
        
        /*
        Load the next trajectory/path. 
        In this example, no splits are used so no split index is needed.
        Alternatively, you can load segment 2 for the next segment of the first path.
        */ 
        final AutoTrajectory pathTwoScore = routine.trajectory("PathTwoTest");

        // Define entry point using routine.active()
        routine.active().onTrue(
                Commands.sequence(
                         // Always reset odometry in first path
                        pathOneScore.resetOdometry(),
                        pathOneScore.cmd(),
                        m_drivetrain.stop().withTimeout(scoreDelayLow), // We needed to add this to our drivetrain
                        pathOneLoad.cmd(),
                        m_drivetrain.stop().withTimeout(scoreDelay),
                        pathTwoScore.cmd()
                ));
        /* 
        Option 1. Add event marker trigger-based behaviors.
        Option 2. Consider using .bind for standard, repetitive commands
        that use common Event Markers which are bound to commands.
        */
        pathOneScore.atTime("scoreL1").onTrue(m_turret.turnClockwise()); // Proof of concept
        pathOneLoad.atTime("scoreL2").onTrue(m_turret.turnCounterClockwise()); // Proof of concept

        return routine;
    }

}
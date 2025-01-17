package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TOFDiagnostics;

public class TOFTestCommand extends Command {
    private final TOFDiagnostics tof;
    private int testPhase = 0;
    private long startTime;
    
    public TOFTestCommand(TOFDiagnostics tof) {
        this.tof = tof;
        addRequirements(tof);
    }

    @Override
    public void initialize() {
        testPhase = 0;
        startTime = System.currentTimeMillis();
        System.out.println("Starting TOF Test Sequence");
    }

    @Override
    public void execute() {
        long currentTime = System.currentTimeMillis();
        long elapsedTime = currentTime - startTime;

        switch(testPhase) {
            case 0: // Test short range
                if (elapsedTime > 0) {
                    System.out.println("Testing Short Range Mode");
                    tof.setShortRange();
                    startTime = currentTime;
                    testPhase++;
                }
                break;

            case 1: // Test medium range
                if (elapsedTime > 2000) {
                    System.out.println("Testing Medium Range Mode");
                    tof.setMediumRange();
                    startTime = currentTime;
                    testPhase++;
                }
                break;

            case 2: // Test long range
                if (elapsedTime > 2000) {
                    System.out.println("Testing Long Range Mode");
                    tof.setLongRange();
                    startTime = currentTime;
                    testPhase++;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return testPhase > 2 && (System.currentTimeMillis() - startTime) > 2000;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TOF Test Sequence Complete");
    }
}
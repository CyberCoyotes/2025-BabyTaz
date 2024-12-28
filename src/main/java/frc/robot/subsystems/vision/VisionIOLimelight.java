package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableEntry;

public class VisionIOLimelight implements VisionIO {
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;

    private synchronized final void updateValues() {
        lastTimeStamp = Timer.getFPGATimestamp() - (pipelineLatency + captureLatency)/1000.0;
        hasTargets = validEntry.getDouble(0.0) == 1.0;
        yawRadians = Units.degreesToRadians(txEntry.getDouble(0.0));
        pitchRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
        botpose = botposeEntry.getDoubleArray(new double[6]);
        tagId = (int)tagIdEntry.getDouble(-1.0);
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.lastTimeStamp = this.lastTimeStamp;
        inputs.horizontalAngleRadians = this.yawRadians;
        inputs.verticalAngleRadians = this.pitchRadians;
        inputs.hasTargets = this.hasTargets;
        inputs.botpose = this.botpose;
        inputs.tagId = this.tagId;
        inputs.pipelineLatency = this.pipelineLatency;
        inputs.captureLatency = this.captureLatency;
    }

    @Override 
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? 3 : 1); // 3=force on, 1=force off
    }
}
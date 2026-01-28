// DriftTestCommand.java
// Purpose: Quantify odometry drift by driving a known pattern and measuring error
// Location: src/main/java/frc/robot/commands/testing/DriftTestCommand.java

package frc.robot.commands.beta;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Drift Test Command - Drives a square pattern to measure odometry accuracy.
 * 
 * PURPOSE:
 * This command helps you understand how much your robot's position estimate
 * drifts over time when using only wheel odometry (no vision correction).
 * 
 * HOW IT WORKS:
 * 1. Records the starting position
 * 2. Drives a square pattern (configurable size)
 * 3. Returns to the starting position
 * 4. Measures the difference between where we think we are vs where we started
 * 
 * HOW TO USE:
 * 1. Place robot at a known position on the field (mark it with tape!)
 * 2. Run this command
 * 3. After it completes, check AdvantageScope for logged drift values
 * 4. Physically measure where the robot actually ended up vs the tape mark
 * 
 * WHAT TO LOOK FOR IN ADVANTAGESCOPE:
 * - DriftTest/CumulativeTranslationDrift: Total X/Y error in meters
 * - DriftTest/CumulativeRotationDrift: Heading error in degrees
 * - DriftTest/PerMeterDrift: Error normalized by distance traveled
 * 
 * @author CyberCoyotes - Educational Drift Testing
 */
@ SuppressWarnings("unused")

public class DriftTestCommand extends Command {
    
    // ============ CONFIGURATION ============
    // Adjust these for your testing space
    
    /** Side length of the square in meters */
    private static final double SQUARE_SIZE_METERS = 2.0;
    
    /** Speed to drive during test (m/s) - keep moderate for consistency */
    private static final double TEST_SPEED = 1.0;
    
    /** How many squares to drive (more = more drift accumulation) */
    private static final int NUM_SQUARES = 2;
    
    /** Pause time at each corner (seconds) - lets robot settle */
    private static final double CORNER_PAUSE = 0.3;
    
    // ============ STATE TRACKING ============
    
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric driveRequest;
    
    private Pose2d startingPose;
    private Pose2d[] cornerPoses;
    private int currentCorner;
    private int currentSquare;
    private Timer segmentTimer;
    private Timer pauseTimer;
    
    private TestPhase phase;
    
    // Accumulated measurements for analysis
    private double totalDistanceTraveled;
    private double maxTranslationError;
    private double maxRotationError;
    
    private enum TestPhase {
        STARTING,
        DRIVING_TO_CORNER,
        PAUSING_AT_CORNER,
        RETURNING_TO_START,
        MEASURING_DRIFT,
        COMPLETE
    }
    
    /**
     * Creates a new drift test command.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     */
    public DriftTestCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.driveRequest = new SwerveRequest.FieldCentric();
        this.segmentTimer = new Timer();
        this.pauseTimer = new Timer();
        
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        // Record where we're starting
        startingPose = drivetrain.getState().Pose;
        
        // Calculate the four corners of our square
        // Square is driven: Forward -> Right -> Back -> Left -> Start
        cornerPoses = new Pose2d[] {
            // Corner 1: Forward
            new Pose2d(
                startingPose.getX() + SQUARE_SIZE_METERS,
                startingPose.getY(),
                startingPose.getRotation()
            ),
            // Corner 2: Forward + Right
            new Pose2d(
                startingPose.getX() + SQUARE_SIZE_METERS,
                startingPose.getY() - SQUARE_SIZE_METERS,
                startingPose.getRotation()
            ),
            // Corner 3: Right (back to start X)
            new Pose2d(
                startingPose.getX(),
                startingPose.getY() - SQUARE_SIZE_METERS,
                startingPose.getRotation()
            ),
            // Corner 4: Back to start
            new Pose2d(
                startingPose.getX(),
                startingPose.getY(),
                startingPose.getRotation()
            )
        };
        
        // Initialize state
        currentCorner = 0;
        currentSquare = 0;
        totalDistanceTraveled = 0;
        maxTranslationError = 0;
        maxRotationError = 0;
        phase = TestPhase.STARTING;
        
        // Log test configuration
        Logger.recordOutput("DriftTest/Config/SquareSize", SQUARE_SIZE_METERS);
        Logger.recordOutput("DriftTest/Config/TestSpeed", TEST_SPEED);
        Logger.recordOutput("DriftTest/Config/NumSquares", NUM_SQUARES);
        Logger.recordOutput("DriftTest/StartPose", startingPose);
        
        System.out.println("=== DRIFT TEST STARTING ===");
        System.out.println("Square size: " + SQUARE_SIZE_METERS + "m");
        System.out.println("Number of squares: " + NUM_SQUARES);
        System.out.println("Starting pose: " + startingPose);
        
        segmentTimer.restart();
        phase = TestPhase.DRIVING_TO_CORNER;
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = drivetrain.getState().Pose;
        
        // Log current state every cycle
        logCurrentState(currentPose);
        
        switch (phase) {
            case DRIVING_TO_CORNER:
                driveTowardsPose(currentPose, cornerPoses[currentCorner]);
                
                // Check if we've reached the corner
                if (isAtPose(currentPose, cornerPoses[currentCorner])) {
                    // Stop and pause
                    stopDrivetrain();
                    pauseTimer.restart();
                    phase = TestPhase.PAUSING_AT_CORNER;
                    
                    // Log corner arrival
                    logCornerArrival(currentCorner, currentSquare, currentPose);
                }
                break;
                
            case PAUSING_AT_CORNER:
                if (pauseTimer.hasElapsed(CORNER_PAUSE)) {
                    // Move to next corner
                    currentCorner++;
                    
                    if (currentCorner >= 4) {
                        // Completed a square
                        currentCorner = 0;
                        currentSquare++;
                        
                        if (currentSquare >= NUM_SQUARES) {
                            // All squares complete - measure final drift
                            phase = TestPhase.MEASURING_DRIFT;
                        } else {
                            phase = TestPhase.DRIVING_TO_CORNER;
                            segmentTimer.restart();
                        }
                    } else {
                        phase = TestPhase.DRIVING_TO_CORNER;
                        segmentTimer.restart();
                    }
                }
                break;
                
            case MEASURING_DRIFT:
                // Calculate and log final drift measurements
                measureAndLogDrift(currentPose);
                phase = TestPhase.COMPLETE;
                break;
                
            case COMPLETE:
                stopDrivetrain();
                break;
                
            default:
                break;
        }
    }
    
    /**
     * Drives toward a target pose using simple proportional control.
     * Keeps heading constant (field-centric square).
     */
    private void driveTowardsPose(Pose2d current, Pose2d target) {
        Translation2d error = target.getTranslation().minus(current.getTranslation());
        double distance = error.getNorm();
        
        // Track total distance
        // (This is approximate - we're sampling periodically)
        totalDistanceTraveled += current.getTranslation()
            .getDistance(drivetrain.getState().Pose.getTranslation());
        
        if (distance > 0.02) {
            // Normalize and scale to test speed
            double vx = (error.getX() / distance) * TEST_SPEED;
            double vy = (error.getY() / distance) * TEST_SPEED;
            
            // Slow down as we approach
            if (distance < 0.3) {
                double slowFactor = distance / 0.3;
                vx *= slowFactor;
                vy *= slowFactor;
            }
            
            drivetrain.setControl(driveRequest
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0));  // Keep heading constant
        } else {
            stopDrivetrain();
        }
    }
    
    /**
     * Checks if we're close enough to consider ourselves "at" a pose.
     */
    private boolean isAtPose(Pose2d current, Pose2d target) {
        double translationError = current.getTranslation()
            .getDistance(target.getTranslation());
        return translationError < 0.05;  // 5cm tolerance
    }
    
    private void stopDrivetrain() {
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
    
    /**
     * Logs state every cycle for detailed analysis.
     */
    private void logCurrentState(Pose2d currentPose) {
        // Current pose
        Logger.recordOutput("DriftTest/CurrentPose", currentPose);
        
        // Error from start (accumulating drift)
        Translation2d translationError = currentPose.getTranslation()
            .minus(startingPose.getTranslation());
        double rotationError = currentPose.getRotation()
            .minus(startingPose.getRotation()).getDegrees();
        
        Logger.recordOutput("DriftTest/Live/TranslationErrorX", translationError.getX());
        Logger.recordOutput("DriftTest/Live/TranslationErrorY", translationError.getY());
        Logger.recordOutput("DriftTest/Live/TranslationErrorMagnitude", translationError.getNorm());
        Logger.recordOutput("DriftTest/Live/RotationErrorDeg", rotationError);
        
        // Phase info
        Logger.recordOutput("DriftTest/Phase", phase.name());
        Logger.recordOutput("DriftTest/CurrentSquare", currentSquare);
        Logger.recordOutput("DriftTest/CurrentCorner", currentCorner);
        
        // Track max errors
        if (translationError.getNorm() > maxTranslationError) {
            maxTranslationError = translationError.getNorm();
        }
        if (Math.abs(rotationError) > maxRotationError) {
            maxRotationError = Math.abs(rotationError);
        }
    }
    
    /**
     * Logs data when arriving at each corner.
     */
    private void logCornerArrival(int corner, int square, Pose2d actualPose) {
        Pose2d expectedPose = cornerPoses[corner];
        Translation2d error = actualPose.getTranslation()
            .minus(expectedPose.getTranslation());
        
        String prefix = "DriftTest/Corner_" + square + "_" + corner + "/";
        Logger.recordOutput(prefix + "ExpectedPose", expectedPose);
        Logger.recordOutput(prefix + "ActualPose", actualPose);
        Logger.recordOutput(prefix + "ErrorX", error.getX());
        Logger.recordOutput(prefix + "ErrorY", error.getY());
        Logger.recordOutput(prefix + "ErrorMagnitude", error.getNorm());
        
        System.out.println("Corner " + corner + " (Square " + square + "): " +
            "Error = " + String.format("%.3f", error.getNorm()) + "m");
    }
    
    /**
     * Final drift measurement and logging.
     * This is the key data for understanding your odometry quality.
     */
    private void measureAndLogDrift(Pose2d finalPose) {
        // Calculate drift from starting position
        Translation2d translationDrift = finalPose.getTranslation()
            .minus(startingPose.getTranslation());
        double rotationDrift = finalPose.getRotation()
            .minus(startingPose.getRotation()).getDegrees();
        
        // Total distance the robot should have traveled
        double expectedDistance = SQUARE_SIZE_METERS * 4 * NUM_SQUARES;
        
        // Drift per meter traveled (key metric!)
        double driftPerMeter = translationDrift.getNorm() / expectedDistance;
        
        // Log all final measurements
        Logger.recordOutput("DriftTest/Final/StartPose", startingPose);
        Logger.recordOutput("DriftTest/Final/EndPose", finalPose);
        Logger.recordOutput("DriftTest/Final/TranslationDriftX", translationDrift.getX());
        Logger.recordOutput("DriftTest/Final/TranslationDriftY", translationDrift.getY());
        Logger.recordOutput("DriftTest/Final/TranslationDriftMagnitude", translationDrift.getNorm());
        Logger.recordOutput("DriftTest/Final/RotationDriftDeg", rotationDrift);
        Logger.recordOutput("DriftTest/Final/ExpectedDistanceTraveled", expectedDistance);
        Logger.recordOutput("DriftTest/Final/DriftPerMeter", driftPerMeter);
        Logger.recordOutput("DriftTest/Final/MaxTranslationError", maxTranslationError);
        Logger.recordOutput("DriftTest/Final/MaxRotationError", maxRotationError);
        
        // Print summary to console
        System.out.println("\n=== DRIFT TEST COMPLETE ===");
        System.out.println("Expected to return to: " + startingPose.getTranslation());
        System.out.println("Actually ended at: " + finalPose.getTranslation());
        System.out.println("Translation drift: " + 
            String.format("%.4f", translationDrift.getNorm()) + " meters");
        System.out.println("  X drift: " + String.format("%.4f", translationDrift.getX()) + " m");
        System.out.println("  Y drift: " + String.format("%.4f", translationDrift.getY()) + " m");
        System.out.println("Rotation drift: " + 
            String.format("%.2f", rotationDrift) + " degrees");
        System.out.println("Total distance traveled: " + expectedDistance + " meters");
        System.out.println("DRIFT PER METER: " + 
            String.format("%.4f", driftPerMeter) + " m/m (" +
            String.format("%.2f", driftPerMeter * 100) + "%)");
        System.out.println("==============================\n");
        
        // Provide interpretation
        if (driftPerMeter < 0.01) {
            System.out.println("✓ EXCELLENT: Less than 1% drift per meter");
        } else if (driftPerMeter < 0.02) {
            System.out.println("✓ GOOD: 1-2% drift per meter - vision will help");
        } else if (driftPerMeter < 0.05) {
            System.out.println("⚠ MODERATE: 2-5% drift - vision fusion recommended");
        } else {
            System.out.println("✗ HIGH DRIFT: >5% - check wheel calibration, tread wear");
        }
    }
    
    @Override
    public boolean isFinished() {
        return phase == TestPhase.COMPLETE;
    }
    
    @Override
    public void end(boolean interrupted) {
        stopDrivetrain();
        
        if (interrupted) {
            System.out.println("Drift test interrupted at square " + 
                currentSquare + ", corner " + currentCorner);
            Logger.recordOutput("DriftTest/Status", "INTERRUPTED");
        } else {
            Logger.recordOutput("DriftTest/Status", "COMPLETE");
        }
    }
}

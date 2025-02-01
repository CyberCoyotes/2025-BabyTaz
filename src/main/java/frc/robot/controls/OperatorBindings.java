package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@SuppressWarnings("unused") // Suppresses unused variable warning

public class OperatorBindings {
    private final CommandXboxController operator;
    
    public OperatorBindings(CommandXboxController operator) {
        this.operator = operator;
        configureBindings();
    }

    private void configureBindings() {
        // Your operator bindings here
        // Face Buttons
        // operatorController.a().onTrue(/* A Button Press */);
        // operatorController.a().whileTrue(/* A Button Hold */);
        // operatorController.b().onTrue(/* B Button Press */);
        // operatorController.b().whileTrue(/* B Button Hold */);
        // operatorController.x().onTrue(/* X Button Press */);
        // operatorController.x().whileTrue(/* X Button Hold */);
        // operatorController.y().onTrue(/* Y Button Press */);
        // operatorController.y().whileTrue(/* Y Button Hold */);

        // Bumpers
        // operatorController.leftBumper().onTrue(/* Left Bumper Press */);
        // operatorController.leftBumper().whileTrue(/* Left Bumper Hold */);
        // operatorController.rightBumper().onTrue(/* Right Bumper Press */);
        // operatorController.rightBumper().whileTrue(/* Right Bumper Hold */);

        // Triggers
        // operatorController.leftTrigger().onTrue(/* Left Trigger Press */);
        // operatorController.leftTrigger().whileTrue(/* Left Trigger Hold */);
        // operatorController.rightTrigger().onTrue(/* Right Trigger Press */);
        // operatorController.rightTrigger().whileTrue(/* Right Trigger Hold */);

        // D-Pad
        // operatorController.povUp().onTrue(/* D-Pad Up Press */);
        // operatorController.povRight().onTrue(/* D-Pad Right Press */);
        // operatorController.povDown().onTrue(/* D-Pad Down Press */);
        // operatorController.povLeft().onTrue(/* D-Pad Left Press */);
        
        // Start/Back
        // operatorController.start().onTrue(/* Start Button Press */);
        // operatorController.back().onTrue(/* Back Button Press */);
    }
}
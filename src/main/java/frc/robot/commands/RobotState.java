/* Create a number status checks related to AprilTags
 * Can be used to determine the robot's current position,
 * provide feedback to the driver, and driveteam
 */
package frc.robot.commands;

public enum RobotState {
    TARGETING_TAG_1("Tag 1 - Loading Station"),
    TARGETING_TAG_2("Tag 2 - Left Score"),
    TARGETING_TAG_3("Tag 3 - Center Score"), 
    TARGETING_TAG_4("Tag 4 - Right Score"),
    NO_TARGET("No Valid Target");

    private final String description;

    RobotState(String description) {
        this.description = description;
    }

    public String getDescription() {
        return description;
    }
}
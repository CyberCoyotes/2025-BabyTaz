package frc.robot.visionV16;

public enum FieldElementLocation {
    // CORAL STATIONS (IDs 1, 2, 12, 13)
    NO_TAG(-1),
    CORAL_STATION_RED_RIGHT(1),
    CORAL_STATION_RED_LEFT(2), 
    CORAL_STATION_BLUE_RIGHT(12),
    CORAL_STATION_BLUE_LEFT(13),

    // PROCESSORS (IDs 3, 16)
    PROCESSOR_RED(3),
    PROCESSOR_BLUE(16),

    // BARGE CENTER (IDs 4, 5, 14, 15) 
    BARGE_RED_OUTER(4),
    BARGE_RED_INNER(5),
    BARGE_BLUE_OUTER(14),
    BARGE_BLUE_INNER(15),

    // REEFS (IDs 6-11 and 17-22)
    REEF_RED_A(6),
    REEF_RED_B(7), 
    REEF_RED_C(8),
    REEF_RED_D(9),
    REEF_RED_E(10),
    REEF_RED_F(11),
    REEF_BLUE_A(17),
    REEF_BLUE_B(18),
    REEF_BLUE_C(19), 
    REEF_BLUE_D(20),
    REEF_BLUE_E(21),
    REEF_BLUE_F(22);

    private final int id;

    FieldElementLocation(int id) {
        this.id = id;
    }

    /**
     * Get the AprilTag ID for this field element
     * @return AprilTag ID
     */
    public int getId() {
        return id;
    }

    /**
     * Get the field element type for a given AprilTag ID
     * @param id AprilTag ID to look up
     * @return The type of field element (REEF, CAGE, etc.) or null if not found
     */
    public static String getElementType(int id) {
        // Look up the enum value for this ID
        for (FieldElementLocation location : values()) {
            if (location.getId() == id) {
                // Extract the base element type from the enum name
                String name = location.name();
                if (name.contains("REEF")) {
                    return "REEF";
                } else if (name.contains("BARGE")) {
                    return "BARGE"; 
                } else if (name.contains("CORAL_STATION")) {
                    return "CORAL_STATION";
                } else if (name.contains("PROCESSOR")) {
                    return "PROCESSOR";
                } else if (name.contains("NO")) {
                    return "Sunk my Battleship!";
                }
            }
        }
        return null;
    }

    /**
     * Check if an AprilTag ID is for a red alliance element
     * @param id AprilTag ID to check
     * @return true if red alliance, false if blue alliance
     */
    public static boolean isRedAlliance(int id) {
        // IDs 1-11 are red alliance
        return id >= 1 && id <= 11;
    }
}
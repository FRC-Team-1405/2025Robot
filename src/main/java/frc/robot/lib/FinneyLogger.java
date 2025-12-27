package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FinneyLogger {
    // Get the NetworkTable instance
    private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    // Get or create a table for logging
    private static final NetworkTable logTable = ntInstance.getTable("AdvantageScopeLogs");

    // Instance-specific log entry
    private final NetworkTableEntry logEntry;

    // Constructor takes a string to determine the log entry key
    public FinneyLogger(String entryKey) {
        this.logEntry = logTable.getEntry(entryKey);
        log("loggerInit");
    }

    // Logs a message to the specific log entry
    public void log(String message) {
        System.out.println(message);
        logEntry.setString(message);
    }

    // Overloaded log method for formatted messages
    public void log(String format, Object... args) {
        String message = String.format(format, args);
        log(message);
    }
}

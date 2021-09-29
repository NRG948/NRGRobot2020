package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * NRG Logging utility methods.
 */
public class Logger {

    private static DriverStation ds = DriverStation.getInstance();

    private static long beginAuto;
    private static long beginTeleop;

    public static void autoStarted() {
        System.out.println("\n<<<<<<<<<<<<<<<<<<< Auto Mode >>>>>>>>>>>>>>>>>>>>\n");
        System.out.println("\n" + DriverStation.getInstance().getEventName() + " " + DriverStation.getInstance().getMatchType().name() + 
                           "\nMatch Number: " + DriverStation.getInstance().getMatchNumber() + 
                           "\nAlliance Color/Position: " + DriverStation.getInstance().getAlliance().name() + DriverStation.getInstance().getLocation());
        beginAuto = System.nanoTime();
    }

    public static void teleopStarted() {
        System.out.println("\n<<<<<<<<<<<<<<<<<<< Teleop Mode >>>>>>>>>>>>>>>>>>>>\n");
        beginTeleop = System.nanoTime();
    }

    public static void commandInit(Command command) {
        commandInit(command, "");
    }

    public static void commandInit(Command command, String details) {
        logModeAndTime(); 
        System.out.printf(" + %s Init %s\n", command.getName(), details);
    }

    public static void commandEnd(Command command, boolean interrupted) {
        commandEnd(command, interrupted, "");
    }

    public static void commandEnd(Command command, boolean interrupted, String details) {
        logModeAndTime();
        System.out.printf(" - %s %s %s\n", command.getName(), interrupted ? "Interrupt" : "End", details);
    }

    public static void event(String s) {
        logModeAndTime();
        System.out.printf(" ! %s\n", s);
    }

    private static void logModeAndTime() {
        if (ds.isAutonomous()) {
            System.out.print("A ");
            logTimeSince(beginAuto);
        } else if (ds.isOperatorControl()) {
            System.out.print("O ");
            logTimeSince(beginTeleop);
        } else if (ds.isTest()) {
            System.out.print("T ");
        } else { // disabled
            System.out.print("D ");
        }
    }

    private static void logTimeSince(long beginTime) {
        double deltaTime = (System.nanoTime() - beginTime) / 1000000000.0;
        System.out.printf("%6.3f ", deltaTime);
    }
}

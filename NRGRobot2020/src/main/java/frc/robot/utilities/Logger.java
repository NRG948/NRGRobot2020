/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Add your docs here.
 */
public class Logger {
    private static long beginAuto;
    private static long beginTeleop;

    public static void autoStarted(){
        System.out.println("<<<<<<<<<<<<<<<<<<< Auto Mode >>>>>>>>>>>>>>>>>>>>\n");
        beginAuto = System.nanoTime();
    }

    public static void teleopStarted(){
        System.out.println("<<<<<<<<<<<<<<<<<<< Teleop Mode >>>>>>>>>>>>>>>>>>>>\n");
        beginTeleop = System.nanoTime();
    }

    public static void commandInit(Command command){
        commandInit(command, "");
    }

    public static void commandInit(Command command, String details){
        logModeAndTime();
        System.out.printf("%s Init %s", command.getName(), details);
    }

    public static void commandEnd(Command command){
        commandEnd(command, "");
    }
    
    public static void commandEnd(Command command, String details){
        logModeAndTime();
        System.out.printf("%s End %s", command.getName(), details);
    }

    private static void logModeAndTime(){
        DriverStation ds = DriverStation.getInstance();
        if (ds.isAutonomous()){
            System.out.print("A");
            logTimeSince(beginAuto);
        } else if(ds.isOperatorControl()){
            System.out.print("O");
            logTimeSince(beginTeleop);
        } else if(ds.isTest()){
            System.out.print("T");
        } else{
            System.out.print("D");
        }
    }

    private static void logTimeSince(long beginTime){
        double deltaTime = (System.nanoTime() - beginTime) / 1000000000;
        System.out.printf(" %6.3f", deltaTime);
    }
}

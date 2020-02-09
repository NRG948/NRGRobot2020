/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;

/**
 * Add your docs here.
 */
public class RaspberryPiPipelines {
    private final RaspberryPiVision raspPi;
    private final PipelineRunner runner;
    public RaspberryPiPipelines(RaspberryPiVision raspPi, PipelineRunner runner) {
        this.raspPi = raspPi;
        this.runner = runner;
    }

    public void initialize() {
        raspPi.setPipelineRunner(runner);
    }

    public void execute() {
    }

    public void end() {

    }

    public boolean isFinished() {
        return true;
    }
}

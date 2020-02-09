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

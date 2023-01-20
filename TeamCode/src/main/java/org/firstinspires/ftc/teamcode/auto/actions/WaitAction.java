package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class WaitAction extends Action {

    private ContinuousAction action;

    public WaitAction(HardwareManager hardware, ContinuousAction action) {
        super(hardware);
        this.action = action;
    }

    @Override
    public void execute() {
        while(!action.isFinished())
        {
            action.execute();
        }
    }
}

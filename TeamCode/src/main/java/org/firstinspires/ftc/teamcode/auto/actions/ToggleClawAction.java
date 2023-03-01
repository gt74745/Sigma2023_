package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class ToggleClawAction extends Action {

    HardwareManager manager;
    boolean isClosed;

    public ToggleClawAction(HardwareManager manager)
    {
        super(manager);
        this.manager = manager;
    }

    private void closeClaw() {
        manager.accessoryServos[0].setPosition(0.58);
        manager.accessoryServos[1].setPosition(0.55);
    }

    private void openClaw() {
        manager.accessoryServos[0].setPosition(0.84);
        manager.accessoryServos[1].setPosition(0.25);
    }


    public void execute()
    {
        isClosed = !isClosed;

        if (isClosed) {
            closeClaw();
        } else
        {
            openClaw();
        }
    }
}

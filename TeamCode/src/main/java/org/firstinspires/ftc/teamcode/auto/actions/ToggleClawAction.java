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

    private void setClawPosition(double val) {
        manager.accessoryServos[0].setPosition(val);
        manager.accessoryServos[1].setPosition(0.91-val);
    }

    public void execute()
    {
        isClosed = !isClosed;

        if (isClosed) {
            setClawPosition(0.82);
        } else
        {
            setClawPosition(0.63);
        }
    }
}

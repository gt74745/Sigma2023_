package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

public class FullStopAction extends Action {
    HardwareManager manager;

    public FullStopAction(HardwareManager manager)
    {
        super(manager);
        this.manager = manager;
    }

    public void execute()
    {
        manager.getLeftBackMotor().setVelocity(0);
        manager.getLeftFrontMotor().setVelocity(0);
        manager.getRightBackMotor().setVelocity(0);
        manager.getRightFrontMotor().setVelocity(0);
    }
}

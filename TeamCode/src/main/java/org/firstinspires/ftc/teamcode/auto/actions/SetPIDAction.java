package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class SetPIDAction extends Action {
    public SetPIDAction(HardwareManager manager, PIDCoefficients coeffs)
    {
        super(manager);
    }
    @Override
    public void execute() {

    }
}

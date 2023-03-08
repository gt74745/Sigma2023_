package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.hardware.HardwareManager;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

public class SetArmAction extends Action {

    private double pos;
    private long delay;

    public SetArmAction(HardwareManager hardware, double pos) {
        super(hardware);
        this.pos = pos;
    }

    public SetArmAction(HardwareManager hardware, double pos, long delay) {
        super(hardware);
        this.pos = pos;
        this.delay = delay;
    }

    private static class DelayArmThread extends Thread {
        @Override
        public void run() {

        }
    }

    @Override
    public void execute() {
        if (delay > 0) {
            ScheduledThreadPoolExecutor exec = new ScheduledThreadPoolExecutor(1);
            exec.schedule(() -> {
                ArmPositionAction.targetArmPos = pos;
            }, this.delay, TimeUnit.MILLISECONDS);
        } else {
            ArmPositionAction.targetArmPos = pos;
        }

    }
}

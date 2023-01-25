package org.firstinspires.ftc.teamcode.auto.actions;

import com.chsrobotics.ftccore.actions.ContinuousAction;
import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.teleop.DriverControl;

public class ArmPositionAction extends ContinuousAction {

    public static double targetArmPos;
    private final PID controller = new PID(new PIDCoefficients(0.002, 0, 0));
    private double prevArmPos;
    private long prevTime;

    public ArmPositionAction(HardwareManager hardware) {
        super(hardware);
    }

    @Override
    public void execute() {
        double armPos = hardware.getLiftMotor().getCurrentPosition();
        double armPosError = targetArmPos - armPos;
        double dArmPosError = (armPos - prevArmPos) / (System.currentTimeMillis() - prevTime);
        double output = controller.getOutput(armPosError, dArmPosError);
//        hardware.accessoryMotors[0].setPower(output);
        prevArmPos = armPos;
        prevTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished()
    {
        return true;
//        return Math.abs(hardware.getLiftMotor().getCurrentPosition() - targetArmPos) < 10;
    }

    @Override
    public void initialize() {
        prevArmPos = hardware.getLiftMotor().getCurrentPosition();
        prevTime = System.currentTimeMillis();
        targetArmPos = 0;
    }
}

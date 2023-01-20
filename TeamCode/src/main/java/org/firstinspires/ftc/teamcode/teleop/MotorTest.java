package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m0 = hardwareMap.dcMotor.get("m0");
        DcMotor m2 = hardwareMap.dcMotor.get("m2");

        waitForStart();

        while (!isStopRequested()) {
            m0.setPower(gamepad1.left_trigger);
            m2.setPower(gamepad1.right_trigger);
        }
    }
}

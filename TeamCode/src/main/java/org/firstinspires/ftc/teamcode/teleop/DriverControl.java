package org.firstinspires.ftc.teamcode.teleop;


import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.teleop.Drive;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;

@TeleOp(name="Sigma TeleOp")
public class DriverControl extends LinearOpMode {
    public static Telemetry telem;
    @Override
    public void runOpMode() throws InterruptedException {
        telem = telemetry;
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c1"))
                .setIMU("imu")
                .setTeleopValues(0.6, 0.6)
                .setOpMode(this)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        UserDriveLoop armLoop = new UserDriveLoop(manager, this) {
            long bLastPressed = 0;
            boolean clawClosed;

            long precisionLastPressed = 0;
            boolean isPrecision;
            boolean isArmOverrideActive;
            double armOverrideOffset = 0;

            private void setLiftPower(double val) {

                telemetry.addData("val", val);
                manager.accessoryMotors[0].setPower(val);
            }

            private void setClawPower(double val) {
                telemetry.addData("val", val);
                manager.accessoryServos[0].setPosition(val);
                manager.accessoryServos[1].setPosition(0.92-val);
            }

            private void closeClaw() {
                manager.accessoryServos[0].setPosition(0.58);
                manager.accessoryServos[1].setPosition(0.55);
            }

            private void openClaw() {
                manager.accessoryServos[0].setPosition(0.78);
                manager.accessoryServos[1].setPosition(0.28);
            }

            @Override
            public void loop() {
                ArmPositionAction armPositionAction = new ArmPositionAction(manager);
                double armPos = manager.accessoryMotors[0].getCurrentPosition();

                if (gamepad1.b) {
                    isArmOverrideActive = true;
                    armOverrideOffset = armPos;
                } else {
                    isArmOverrideActive = false;
                }

                if (gamepad1.right_trigger > 0.1)
                {
                    if (armPos - armOverrideOffset < 2900 || isArmOverrideActive)
                        setLiftPower(gamepad1.right_trigger);
                    else
                        armPositionAction.execute();
                } else if (gamepad1.left_trigger > 0.1)
                {
                    if (armPos - armOverrideOffset > 0 || isArmOverrideActive)
                        setLiftPower(-gamepad1.left_trigger);
                    else
                        armPositionAction.execute();
                } else
                {
                    armPositionAction.execute();
                }
                telemetry.addData("l0", manager.accessoryMotors[0].getCurrentPosition());
                telemetry.addData("c0", clawClosed);
                telemetry.addData("bDiff", System.currentTimeMillis() - bLastPressed);
                telemetry.update();


                ArmPositionAction.targetArmPos = armPos;

                if (gamepad1.right_bumper && System.currentTimeMillis() - bLastPressed > 250) {
                    clawClosed = !clawClosed;
                    bLastPressed = System.currentTimeMillis();
                }

                if (clawClosed) {
                    closeClaw();
                } else
                {
                    openClaw();
                }
//                telemetry.update();
                if (gamepad1.x && System.currentTimeMillis() - precisionLastPressed > 250) {
                    isPrecision = !isPrecision;
                    precisionLastPressed = System.currentTimeMillis();
                }

                if (isPrecision) {
                    manager.linearSpeed = 0.8;
                    manager.rotSpeed = 0.55;
                } else
                {
                    manager.linearSpeed = 1;
                    manager.rotSpeed = 0.55;
                }

                if (gamepad1.a)
                {
                    manager.IMUReset = manager.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                }
            }
        };

        Drive drive = new Drive.Builder(manager)
                .addUserLoop(armLoop)
                .build();

        waitForStart();

        drive.runDriveLoop();
    }
}

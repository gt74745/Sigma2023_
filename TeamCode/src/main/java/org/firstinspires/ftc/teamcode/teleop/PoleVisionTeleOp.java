package org.firstinspires.ftc.teamcode.teleop;


import com.chsrobotics.ftccore.engine.navigation.path.MotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.teleop.Drive;
import com.chsrobotics.ftccore.teleop.UserDriveLoop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.PoleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Vision TeleOp")
public class PoleVisionTeleOp extends LinearOpMode {
    public static Telemetry telem;
    private DcMotorEx lat;
    private DcMotorEx lon;

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeHolder.opMode = this;
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

        int cameraMonitorViewId = manager.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", manager.hardwareMap.appContext.getPackageName());
        WebcamName poleDetectionWebcam = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvWebcam poleWebcam = OpenCvCameraFactory.getInstance().createWebcam(poleDetectionWebcam, cameraMonitorViewId);
        OpenCvPipeline polePipeline = new PoleDetectionPipeline();

        poleWebcam.setPipeline(polePipeline);
        poleWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                poleWebcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Pole detection camera open failed with code", errorCode);
                telemetry.update();
            }
        });

        lat = hardwareMap.get(DcMotorEx.class, "odo0");
        lat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lat.setDirection(DcMotorSimple.Direction.FORWARD);
        lat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lon = hardwareMap.get(DcMotorEx.class, "odo1");
        lon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lon.setDirection(DcMotorSimple.Direction.FORWARD);
        lon.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lon.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        UserDriveLoop armLoop = new UserDriveLoop(manager, this) {
            long bLastPressed = 0;
            boolean clawClosed;

            long bumperLastPressed = 0;
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

            @Override
            public void loop() {
                ArmPositionAction armPositionAction = new ArmPositionAction(manager);
                double armPos = manager.accessoryMotors[0].getCurrentPosition();

                if (gamepad1.left_bumper) {
                    isArmOverrideActive = true;
                } else if (isArmOverrideActive) {
                    isArmOverrideActive = false;
                    armOverrideOffset = armPos;
                }

                if (gamepad1.right_trigger > 0.1)
                {
                    if (armPos - armOverrideOffset < 4000 || isArmOverrideActive)
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
//                telemetry.addData("l0", manager.accessoryMotors[0].getCurrentPosition());
//                telemetry.addData("c0", clawClosed);
//                telemetry.addData("bDiff", System.currentTimeMillis() - bLastPressed);

                ArmPositionAction.targetArmPos = armPos;

                if (gamepad1.b && System.currentTimeMillis() - bLastPressed > 250) {
                    clawClosed = !clawClosed;
                    bLastPressed = System.currentTimeMillis();
                }

                if (clawClosed) {
//                    setClawPower(0.85);
                } else
                {
//                    setClawPower(0.58);
                }
                telemetry.update();
                if (gamepad1.right_bumper && System.currentTimeMillis() - bumperLastPressed > 250) {
                    isPrecision = !isPrecision;
                    bumperLastPressed = System.currentTimeMillis();
                }

                if (isPrecision) {
                    manager.linearSpeed = 0.3;
                    manager.rotSpeed = 0.3;
                } else
                {
                    manager.linearSpeed = 0.8;
                    manager.rotSpeed = 0.6;
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


//    public void navigateInALinearFashion(Position destination)
//    {
//        position = localization.getCurrentPosition();
//
//        double orientation = Math.atan2(destination.y - position.y, destination.x - position.x) - Math.PI / 4 - position.t;
//
//        time = System.currentTimeMillis();
//
//        if (profile != null && hardware.profileCtrler != null)
//            magnitude = hardware.profileCtrler.getOutput(error, (error - lastError) / (time - lastTime));
//
//        magnitude = linearController.getOutput(error, (error - lastError) / (time - lastTime));
//
//        double negOutput = magnitude * Math.sin(orientation);
//
//        double posOutput = magnitude * Math.cos(orientation);
//        if (orientation == 0) {
//            posOutput = negOutput;
//        }
//
//        double thetaOutput = Math.abs(thetaError) >= hardware.tolerances.rotational ? rotationController.getOutput(Math.abs(thetaError), 0) : 0;
//
//        lastTime = time;
//        lastError = error;
//
//        if (hardware.debugMode) {
//            hardware.opMode.telemetry.addData("error", error);
//            hardware.opMode.telemetry.addData("direction", orientation);
//            hardware.opMode.telemetry.update();
//        }
//
//        hardware.getLeftFrontMotor().setVelocity((-posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
//        hardware.getRightFrontMotor().setVelocity((negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
//        hardware.getLeftBackMotor().setVelocity((-negOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
//        hardware.getRightBackMotor().setVelocity((posOutput) + ((isCounterClockwise ? 1 : -1) * thetaOutput));
//    }
//
//    public Position getRobotPosition() {
//        //Encoder values. These are in ticks. We will later convert this to a usable distance.
//
//        //Record encoder values.
//        int lat = hardware.accessoryOdometryPods[0].getCurrentPosition();
//        int lon = hardware.accessoryOdometryPods[1].getCurrentPosition();
//
//        double heading = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - hardware.offset;
//
//        double d = Math.abs(heading - lastHeading) % (2 * Math.PI);
//        double r = d > Math.PI ? (2 * Math.PI) - d : d;
//
//        int sign = (heading - lastHeading >= 0 && heading - lastHeading <= Math.PI) || (heading - lastHeading <= -Math.PI && heading - lastHeading >= -2 * Math.PI) ? 1 : -1;
//        r *= sign;
//
//        //Calculate displacement values in mm
//        //Account for active rotation by subtracting the arc length of heading displacement
//        double d_lat = ((lat - lastLat) * mmPerTick) - (hardware.latWheelOffset * r);
//        double d_lon = ((lon - lastLon) * mmPerTick) - (hardware.lonWheelOffset * r);
//
//        //Account for current orientation
//        double dx = d_lat * Math.cos(heading) + d_lon * Math.sin(heading);
//        double dy = d_lon * Math.cos(heading) - d_lat * Math.sin(heading);
//
//        //Store encoder values so we can use them in calculating displacement.
//        lastLat = lat;
//        lastLon = lon;
//        lastHeading = heading;
//
//        //Update robot position
//        Position robotPosition = new Position();
//        robotPosition.x = previousPosition.x - dx;
//        robotPosition.y = previousPosition.y + dy;
//        robotPosition.t = heading;
//
//        hardware.opMode.telemetry.addData("lat", lat);
//        hardware.opMode.telemetry.addData("lon", lon);
//        hardware.opMode.telemetry.addData("heading", heading);
//
//        if (robotPosition.x == 0) {
//            robotPosition.x = 0.0000001;
//        }
//        log.addData(robotPosition.x + "," + robotPosition.y + "," + robotPosition.t + "," + hardware.accessoryOdometryPods[0].getCurrentPosition() + "," + hardware.accessoryOdometryPods[1].getCurrentPosition() + "," + d + "," + r + "," + dx + "," + dy + "," + previousPosition.x + "," + previousPosition.y + "," + previousPosition.t);
//        log.update();
//
//        return robotPosition;
//    }
}

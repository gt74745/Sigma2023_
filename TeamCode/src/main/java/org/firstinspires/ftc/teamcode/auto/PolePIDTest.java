package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.util.Homography;
import org.firstinspires.ftc.teamcode.auto.util.OpModeHolder;
import org.firstinspires.ftc.teamcode.auto.util.PoleDetectionPipeline;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name = "Pole PID Test")
public class PolePIDTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        OpModeHolder.opMode = this;
        Config config = new Config.Builder()
                .setDebugMode(true)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
//                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
//                .addAccessory(new Accessory(AccessoryType.SERVO, "c0"))
//                .addAccessory(new Accessory(AccessoryType.SERVO, "c1"))
//                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo1"))
                .setOdometryWheelProperties(8192, 35, -233.2037353515/2, -186.0614013671/2)
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(4.3, 0.00003, 8.0), new PIDCoefficients(700, 0.005, 0))
                .setNavigationTolerances(new Tolerances(45, 0.1))
                .setHighPrecisionTolerances(new Tolerances(17, 0.09))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryOdometryPods[0].setDirection(DcMotorSimple.Direction.REVERSE);
        manager.accessoryOdometryPods[1].setDirection(DcMotorSimple.Direction.FORWARD);

        manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = manager.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", manager.hardwareMap.appContext.getPackageName());
        WebcamName poleDetectionWebcam = hardwareMap.get(WebcamName.class, "webcam");
        OpenCvWebcam poleWebcam = OpenCvCameraFactory.getInstance().createWebcam(poleDetectionWebcam, cameraMonitorViewId);
        PoleDetectionPipeline polePipeline = new PoleDetectionPipeline();

        poleWebcam.setPipeline(polePipeline);
        poleWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                poleWebcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Pole detection camera open failed with code", errorCode);
                telemetry.update();
            }
        });


        waitForStart();

        Point offset = new Point();
        Rect avg = polePipeline.averageRect;
        if (avg != null && avg.width != 0 && avg.height > 20) {
            Point bottomMid = new Point(avg.x + avg.width/2d, avg.y + avg.height);
            offset = Homography.positionFromPoint(bottomMid);
            OpModeHolder.opMode.telemetry.addData("Rx", avg.x);
            OpModeHolder.opMode.telemetry.addData("Ry", avg.y);
            OpModeHolder.opMode.telemetry.addData("Rw", avg.width);
            OpModeHolder.opMode.telemetry.addData("Rh", avg.height);
            OpModeHolder.opMode.telemetry.addData("Mx", bottomMid.x);
            OpModeHolder.opMode.telemetry.addData("My", bottomMid.y);
            OpModeHolder.opMode.telemetry.addData("ox", offset.x);
            OpModeHolder.opMode.telemetry.addData("oy", offset.y);
            OpModeHolder.opMode.telemetry.update();
        }

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addLinearPath(PrecisionMode.HIGH, new Position(offset.x, offset.y, 0))
                .build();

        pipeline.execute();
    }
}

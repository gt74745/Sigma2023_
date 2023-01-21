package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
import com.chsrobotics.ftccore.vision.CVUtility;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.auto.actions.ArmPositionAction;
import org.firstinspires.ftc.teamcode.auto.actions.DelayAction;
import org.firstinspires.ftc.teamcode.auto.actions.FullStopAction;
import org.firstinspires.ftc.teamcode.auto.actions.SetArmAction;
import org.firstinspires.ftc.teamcode.auto.actions.ToggleClawAction;
import org.firstinspires.ftc.teamcode.auto.actions.WaitAction;
import org.firstinspires.ftc.teamcode.auto.util.SignalSleeveDetector;

@Autonomous(name = "Left Side")
public class LeftAuton extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Config config = new Config.Builder()
                .setDebugMode(true)
                .setDriveMotors("m0", "m1", "m2", "m3")
                .setMotorDirection(DcMotorSimple.Direction.FORWARD)
                .addAccessory(new Accessory(AccessoryType.MOTOR, "l0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c0"))
                .addAccessory(new Accessory(AccessoryType.SERVO, "c1"))
                .addAccessory(new Accessory(AccessoryType.WEBCAM, "webcam"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo1"))
                .setOdometryWheelProperties(8192, 70, -233, -186)
//                .setOdometryWheelProperties(8192, 70, -233.2037353515, -186.0614013671)
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(4.3, 0.00004, 8.0), new PIDCoefficients(700, 0.01, 0))
                .setNavigationTolerances(new Tolerances(45, 0.20))
                .setHighPrecisionTolerances(new Tolerances(17, 0.09))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryOdometryPods[0].setDirection(DcMotorSimple.Direction.REVERSE);
        manager.accessoryOdometryPods[1].setDirection(DcMotorSimple.Direction.FORWARD);

        manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        
        CVUtility cv = null;
        try {
            cv = new CVUtility(manager, telemetry);
        } catch (Exception e) {
            telemetry.addLine("CVUtility failed to initialized");
            telemetry.update();
        }

        ArmPositionAction armPositionAction = new ArmPositionAction(manager);
        ToggleClawAction toggleClawAction = new ToggleClawAction(manager);
        toggleClawAction.execute();

        waitForStart();

        int dots = 1;
        if (cv != null && cv.initialized && cv.grabFrame() != null) {
            dots = SignalSleeveDetector.detectOrientation(cv.grabFrame());

            telemetry.addData("Dots: ", dots);
            cv.stopStreaming();
        } else {
            telemetry.addLine("Signal sleeve detection failed");
        }
        double parkingPos;
        parkingPos = dots == 1 ? -570 :
                (dots == 2 ? 0 : 600);

        telemetry.update();

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(armPositionAction)
                .addAction(new SetArmAction(manager, 3000))
                .addLinearPath(
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(600, 75, 0),
                        new Position(620, 1300, 0)
                )
                .addAction(new SetArmAction(manager, 4100))
                .addLinearPath(
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
//                        new Position(-480, 1440, 7 * Math.PI / 4)
                        new Position(285, 1340, 0)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, 3900))
                .addAction(toggleClawAction)                                    // Drop cone 1
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 700))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(-610, 1300, Math.PI / 2, 0.9)
                )
                .addAction(new FullStopAction(manager))
                .addAction(toggleClawAction)                                    // Pickup cone 2
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager,4100))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(                                                 // Align with high junction
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(125, 1420, 7 * Math.PI / 4, 0.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, 3900))
                .addAction(toggleClawAction)                                    // Drop cone 2
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 500))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(-615, 1295, Math.PI / 2, 0.9)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)                                    // Pickup cone 3
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager, 4100))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(                                                 // Align with high junction
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(130, 1420, 7 * Math.PI / 4, 0.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, 3900))
                .addAction(toggleClawAction)                                    // Drop cone 3
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 500))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(-610, 1300, Math.PI / 2, 0.9)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .addAction(toggleClawAction)                                    // Pickup cone 4
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager, 4100))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(                                                 // Align with high junction
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(145, 1420, 7 * Math.PI / 4, 0.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, 3900))
                .addAction(toggleClawAction)                                    // Drop cone 4
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(                                                 // Park
                        PrecisionMode.HIGH,
                        new TrapezoidalMotionProfile(900, 1400),
                        new Position(parkingPos, 1200, 0)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .build();

        pipeline.execute();
    }
}
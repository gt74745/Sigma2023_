package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.actions.Action;
import com.chsrobotics.ftccore.engine.navigation.path.PrecisionMode;
import com.chsrobotics.ftccore.engine.navigation.path.Tolerances;
import com.chsrobotics.ftccore.engine.navigation.path.TrapezoidalMotionProfile;
import com.chsrobotics.ftccore.geometry.Position;
import com.chsrobotics.ftccore.hardware.HardwareManager;
import com.chsrobotics.ftccore.hardware.config.Config;
import com.chsrobotics.ftccore.hardware.config.accessory.Accessory;
import com.chsrobotics.ftccore.hardware.config.accessory.AccessoryType;
import com.chsrobotics.ftccore.pipeline.Pipeline;
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
import org.firstinspires.ftc.teamcode.auto.util.AprilTagSleeveDetector;

import java.util.ArrayList;

@Autonomous(name = "Right All Medium")
public class RightAllMediumPoleAuton extends LinearOpMode
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
//                .setOdometryWheelProperties(8192, 70, -233, -186)
                .setOdometryWheelProperties(8192, 35, -233.2037353515/2, -186.0614013671/2)
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(3.5, 0.008, 20.0), new PIDCoefficients(700, 0.003, 0))
                .setNavigationTolerances(new Tolerances(45, 0.1))
                .setHighPrecisionTolerances(new Tolerances(17, 0.04))
                .setLowPrecisionTolerances(new Tolerances(45, 0.1))
                .setRotationMovementCoefficient(1/4000d)
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        manager.accessoryOdometryPods[0].setDirection(DcMotorSimple.Direction.REVERSE);
        manager.accessoryOdometryPods[1].setDirection(DcMotorSimple.Direction.FORWARD);

        manager.driveMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        manager.driveMotors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        AprilTagSleeveDetector sleeveDetector = new AprilTagSleeveDetector(manager);

        AprilTagSleeveDetector.Zone zone = AprilTagSleeveDetector.Zone.ZONE_TWO;

        ArmPositionAction armPositionAction = new ArmPositionAction(manager);
        ToggleClawAction toggleClawAction = new ToggleClawAction(manager);
        toggleClawAction.execute();

        while (!isStarted() && !isStopRequested())
            sleeveDetector.detect(manager);


        waitForStart();

        zone = sleeveDetector.zone;
        int dots = 1;
        if (zone != null) {
            if (zone.equals(AprilTagSleeveDetector.Zone.ZONE_TWO)) {
                dots = 2;
            } else if (zone.equals(AprilTagSleeveDetector.Zone.ZONE_THREE)) {
                dots = 3;
            }
        }

        if (sleeveDetector.camera.getFps() != 0) {
            sleeveDetector.camera.stopStreaming();
        }

        double parkingPos = dots == 1 ? -570 :
                (dots == 2 ? 0 : 600);

        double maxLiftHeight = 2600;

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(800, 1500);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(armPositionAction)
                .addAction(new SetArmAction(manager, 1500))
                .addCurvedPath(
                        new TrapezoidalMotionProfile(900, 1700),
                        new ArrayList<Action>() {{
                            add(new SetArmAction(manager, maxLiftHeight));
                        }},
                        new ArrayList<Double>() {{
                            add(0.7);
                        }},
                        new Position(0, 0, 0),
                        new Position(-350, 150, 0),
                        new Position(-450, 500, 0),
                        new Position(-500, 700, 0),
                        new Position(-550, 1000, 0)
//                        new Position(-325, 1200, 0),
//                        new Position(-100, 1365, 0)
                )
                .addLinearPath(PrecisionMode.HIGH, new Position(-530, 1000, 3 * Math.PI/2, 0))
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, 2200))
                .addAction(new DelayAction(manager, 200))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 400))
                // todo: make these two moves a single spline
                .addLinearPath(                                                 // Align with cone stack y
                        PrecisionMode.LOW,
                        profile,
                        new Position(-530, 1295, 3 * Math.PI/2)
                )
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        profile,
                        new Position(555, 1295, 3 * Math.PI / 2, 0, 5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(toggleClawAction)                                    // Pickup cone 2
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager,maxLiftHeight - 700))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(PrecisionMode.HIGH, profile, new Position(-310, 1265, Math.PI, 1.4))
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, maxLiftHeight - 1000))
                .addAction(new DelayAction(manager, 200))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 350))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        profile,
                        new Position(565, 1275, 3 * Math.PI / 2, 0.8, 3.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(toggleClawAction)                                    // Pickup cone 3
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager,maxLiftHeight - 700))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(PrecisionMode.HIGH, profile, new Position(-310, 1265, Math.PI, 1.4))
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, maxLiftHeight - 1000))
                .addAction(new DelayAction(manager, 200))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 250))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        profile,
                        new Position(555, 1285, 3 * Math.PI / 2, 0.8, 3.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(toggleClawAction)                                    // Pickup cone 4
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager,maxLiftHeight - 700))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(PrecisionMode.HIGH, profile, new Position(-320, 1265, Math.PI, 1.4))
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, maxLiftHeight - 1000))
                .addAction(new DelayAction(manager, 200))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 200))
                .addLinearPath(                                                 // Align with cone stack
                        PrecisionMode.HIGH,
                        profile,
                        new Position(545, 1285, 3 * Math.PI / 2, 0.8, 3.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(toggleClawAction)                                    // Pickup cone 5
                .addAction(new DelayAction(manager, 400))
                .addAction(new SetArmAction(manager,maxLiftHeight - 700))
                .addAction(new DelayAction(manager, 200))
                .addLinearPath(PrecisionMode.HIGH, profile, new Position(-340, 1265, Math.PI, 1.4))
                .addAction(new FullStopAction(manager))
                .addAction(new SetArmAction(manager, maxLiftHeight - 1000))
                .addAction(new DelayAction(manager, 200))
                .addAction(toggleClawAction)
                .addAction(new DelayAction(manager, 200))
                .addAction(new SetArmAction(manager, 0))
                .addLinearPath(                                                 // Park
                        PrecisionMode.HIGH,
                        profile,
                        new Position(parkingPos, 1250, 0, 0.5)
                )
                .addAction(new FullStopAction(manager))
                .addAction(new WaitAction(manager, armPositionAction))
                .build();

        pipeline.execute();
    }
}

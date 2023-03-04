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

@Autonomous(name = "Right Park")
public class RightParkAuton extends LinearOpMode
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
                .setPIDCoefficients(new PIDCoefficients(3.5, 0.008, 4.0), new PIDCoefficients(700, 0.003, 0))
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

        double parkingPos = dots == 1 ? -500 :
                (dots == 2 ? 200 : 700);

        double maxLiftHeight = 2600;

        double poleLiftHeight = 2700;

        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(800, 1500);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addContinuousAction(armPositionAction)
                .addLinearPath(
                        profile,
                        new Position(-300, 0, 0),
                        new Position(-300, 1250, 0),
                        new Position(parkingPos, 1250, 0)
                )
                .addAction(new FullStopAction(manager))
                .build();

        pipeline.execute();
    }
}

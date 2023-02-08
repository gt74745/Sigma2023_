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

@Autonomous(name = "Spline Test")
public class SplineTestAuton extends LinearOpMode
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
                .setPIDCoefficients(new PIDCoefficients(4.3, 0.00003, 8.0), new PIDCoefficients(650, 0.005, 0))
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

        manager.accessoryMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        Pipeline pipeline = new Pipeline.Builder(manager)
                .addCurvedPath(
                        new Position(0,0,0),
                        new Position(727, 360, 0),
                        new Position(-727, 720, 0),
                        new Position(0, 1080, 0),
                        new Position(0, 0, 0)
                )
//                .addLinearPath(new Position(1000, 0, 0))
                .build();

        pipeline.execute();
    }
}

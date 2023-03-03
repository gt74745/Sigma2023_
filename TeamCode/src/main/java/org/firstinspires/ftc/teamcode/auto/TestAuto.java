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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Disabled
@Autonomous(name="Test")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Config config = new Config.Builder()
                .setDriveMotors("m0", "m1", "m2", "m3")
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo0"))
                .addAccessory(new Accessory(AccessoryType.ODOMETRY_POD, "odo1"))
                .setOdometryWheelProperties(8192, 70, 8, -8)
                .setOpMode(this)
                .setIMU("imu")
                .setPIDCoefficients(new PIDCoefficients(45, 0.0002, 80), new PIDCoefficients(750, 0.006, 0))
                .setNavigationTolerances(new Tolerances(10, 0.02))
                .build();

        HardwareManager manager = new HardwareManager(config, hardwareMap);

        Pipeline pipeline = new Pipeline.Builder(manager)
                .addLinearPath(
                        new TrapezoidalMotionProfile(60, 120),
                        new Position(0, 24, 0),
                        new Position(24, 24, Math.PI / 2)
                )
                .addCurvedPath(
                        new Position(24, 24, Math.PI / 2),
                        new Position(48, 24, Math.PI / 2),
                        new Position(48, 64, Math.PI / 2),
                        new Position(84, 64, Math.PI / 2)
                )
                .build();

        waitForStart();

        pipeline.execute();
    }
}

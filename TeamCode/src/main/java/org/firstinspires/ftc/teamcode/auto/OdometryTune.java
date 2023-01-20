package org.firstinspires.ftc.teamcode.auto;

import com.chsrobotics.ftccore.engine.navigation.control.PID;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Odometry Offset Tuner")
public class OdometryTune extends LinearOpMode
{
    double error, thetaError;
    boolean isCounterClockwise;

    @Override
    public void runOpMode() throws InterruptedException
    {
        DcMotorEx m0 = hardwareMap.get(DcMotorEx.class, "m0");
        DcMotorEx m1 = hardwareMap.get(DcMotorEx.class, "m1");
        DcMotorEx m2 = hardwareMap.get(DcMotorEx.class, "m2");
        DcMotorEx m3 = hardwareMap.get(DcMotorEx.class, "m3");

        DcMotor odo0 = hardwareMap.dcMotor.get("odo0");
        DcMotor odo1 = hardwareMap.dcMotor.get("odo1");

        odo0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odo0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odo1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(params);

        waitForStart();

        while (!isStopRequested())
        {
            double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
            calculateError(Math.PI / 2, heading);
            PID pid = new PID(new PIDCoefficients(750, 0.03, 0));
            double output = pid.getOutput(Math.abs(thetaError), 0);
            double power = (isCounterClockwise ? 1 : -1) * output;

            m0.setVelocity(power);
            m1.setVelocity(power);
            m2.setVelocity(power);
            m3.setVelocity(power);

            telemetry.addData("odo0", odo0.getCurrentPosition() * 4);
            telemetry.addData("odo1", odo1.getCurrentPosition() * 4);
            telemetry.update();
        }
    }
    public void calculateError(double targetT, double t)
    {
        thetaError = targetT - t;

        isCounterClockwise = false;

        if (Math.abs(targetT - (t - (2 * Math.PI))) < Math.abs(thetaError))
        {
            thetaError = targetT - (t - (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (Math.abs(targetT - (t + (2 * Math.PI))) < thetaError)
        {
            thetaError = targetT - (t + (2 * Math.PI));
            isCounterClockwise = true;
        }

        if (thetaError > 0 && (thetaError < Math.PI))
            isCounterClockwise = true;

        if (thetaError < 0 && (thetaError > -Math.PI))
            isCounterClockwise = false;

//        return (error < hardware.tolerances.linear && Math.abs(thetaError) < hardware.tolerances.rotational);
    }
}

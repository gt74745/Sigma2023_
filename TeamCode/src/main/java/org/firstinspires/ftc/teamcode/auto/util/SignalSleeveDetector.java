package org.firstinspires.ftc.teamcode.auto.util;

import android.graphics.Path;

import com.chsrobotics.ftccore.hardware.HardwareManager;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class SignalSleeveDetector {
//        int cameraMonitorViewId = OpModeHolder.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", OpModeHolder.opMode.hardwareMap.appContext.getPackageName());
//        OpenCvCamera cvCamera = OpenCvCameraFactory.getInstance().createWebcam(manager.getWebcam(), cameraMonitorViewId);


    public static int detectOrientation(Mat mat) {
        if (mat == null) {
            return 1;
        }

        // Do color detection
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat green = new Mat();
        Core.inRange(hsvMat, new Scalar(35, 40, 35), new Scalar(70, 120, 255), green);
        int greenIndex = 0;
        for (int i = green.rows() / 3; i < green.rows() * 2 / 3; i += 5) {
            for (int j = green.cols() / 3; j < green.cols() * 2 / 3; j += 5) {
                if (green.get(i,j)[0] != 0) {
                    greenIndex++;
                }
            }
        }
        Mat blue = new Mat();
        Core.inRange(hsvMat, new Scalar(75, 80, 35), new Scalar(130, 170, 255), blue);
        int blueIndex = 0;
        for (int i = blue.rows() / 3; i < blue.rows() * 2 / 3; i += 5) {
            for (int j = blue.cols() / 3; j < blue.cols() * 2 / 3; j += 5) {
                if (blue.get(i,j)[0] != 0) {
                    blueIndex++;
                }
            }
        }

        int threshold = 200;
//        OpModeHolder.opMode.telemetry.update();
        if (greenIndex > threshold || blueIndex > threshold) {
            if (greenIndex > blueIndex) {
                return 2;
            }
            return 3;
        }
        return 1;
    }

}

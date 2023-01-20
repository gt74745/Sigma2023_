package org.firstinspires.ftc.teamcode.auto.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WebcamPipeline extends OpenCvPipeline {

    private static Mat lastMat;

    @Override
    public Mat processFrame(Mat input) {
        lastMat = input;

//        Mat hsvMat = new Mat();
//        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
//        Mat green = new Mat();
//        Core.inRange(hsvMat, new Scalar(75, 80, 35), new Scalar(135, 170, 255), green);

        return input;
    }

    public static Mat getLastMat() {
        return lastMat;
    }
    public static void clearLastMat() {
        lastMat = null;
    }
}

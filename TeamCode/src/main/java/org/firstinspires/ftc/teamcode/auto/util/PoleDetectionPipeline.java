package org.firstinspires.ftc.teamcode.auto.util;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class PoleDetectionPipeline extends OpenCvPipeline {

    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;
    public double rx;
    public double ry;
    public double rw;
    public double rh;

    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // todo: analyze multiple frames to improve consistency

        // release color detection mats
        input.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
        scaledMask.release();
        masked.release();
        edges.release();
        finalMask.release();
        hierarchy.release();

        // Find largest connected component as a rectangle
        Mat labeled = new Mat(thresh.size(), thresh.type());
        Mat rectComponents = Mat.zeros(new Size(0, 0), 0);
        Mat centComponents = Mat.zeros(new Size(0, 0), 0);

        Imgproc.connectedComponentsWithStats(thresh, labeled, rectComponents, centComponents);

        // Collect regions info
        int[] rectangleInfo = new int[5];

        Rect largestRect = null;
        for(int i = 1; i < rectComponents.rows(); i++) {
            // Extract bounding box
            rectComponents.row(i).get(0, 0, rectangleInfo);
            Rect rectangle = new Rect(rectangleInfo[0], rectangleInfo[1], rectangleInfo[2], rectangleInfo[3]);
            if (largestRect == null || rectangle.area() > largestRect.area()) {
                largestRect = rectangle;
            }
        }

        if (largestRect != null) {
            rx = largestRect.x;
            ry = largestRect.y;
            rw = largestRect.width;
            rh = largestRect.height;
            int center = 600 - 2 * (int) rw;
            OpModeHolder.opMode.telemetry.addData("Rx", largestRect.x);
            OpModeHolder.opMode.telemetry.addData("Ry", largestRect.y);
            OpModeHolder.opMode.telemetry.addData("Rw", largestRect.width);
            OpModeHolder.opMode.telemetry.addData("Rh", largestRect.height);
            OpModeHolder.opMode.telemetry.addData("Rmid", largestRect.x + largestRect.width/2);
            OpModeHolder.opMode.telemetry.addData("center", center);
            OpModeHolder.opMode.telemetry.update();
        } else {
            rx = 0;
            ry = 0;
            rw = 0;
            rh = 0;
        }

        rectComponents.release();
        centComponents.release();
        labeled.release();
        thresh.release();


        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
//        return contours.size() != 0 ? contours.get(0) : finalMask;
        return mat;
    }

}
